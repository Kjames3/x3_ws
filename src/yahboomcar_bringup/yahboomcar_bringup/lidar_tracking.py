"""Diagnostic temporal gate and bounded local reacquisition over frozen geometry.

No hardware/ROS access. All evidence uses acquisition stamps. Recovery proposals
consume a frozen accumulated window; verification consumes strictly later
clouds. A bounded local search cannot establish global uniqueness.
"""
from collections import deque
from dataclasses import dataclass, replace
from copy import copy
import numpy as np
from .lidar_registration import pose_matrix, transform


def yaw(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))


def angle(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


def distance(A, B, odom):
    a, b = A @ odom, B @ odom
    return float(np.linalg.norm(a[:2, 3]-b[:2, 3])), abs(angle(yaw(a)-yaw(b)))


@dataclass(frozen=True)
class Policy:
    window: int = 5
    support: int = 4
    min_span_s: float = .35
    pitch_span_rad: float = np.deg2rad(10.)
    radius_m: float = .06
    radius_yaw_rad: float = np.deg2rad(2.)
    evidence_max_age_s: float = 1.
    lost_after_s: float = 1.5
    lost_after_failures: int = 5
    search_xy_m: float = .6
    search_yaw_rad: float = np.deg2rad(20.)
    max_hypotheses: int = 3
    verification_frames: int = 5
    verification_limit: int = 12
    score_margin: float = .08


class Tracker:
    def __init__(self, matcher, policy=None):
        self.matcher = matcher
        self.policy = policy or Policy()
        p = self.policy
        if not (2 <= p.support <= p.window and p.verification_frames >= 2):
            raise ValueError('invalid evidence window')
        self.recovery_matcher = copy(matcher)  # share immutable map/KD tree
        self.recovery_matcher.limits = replace(matcher.limits,
            max_points=2000, max_iterations=30, correspondence_m=.6,
            max_correction_m=1.2, max_correction_yaw_rad=np.deg2rad(40.))
        self.correction = None
        self.state = 'UNINITIALIZED'
        self.last_stamp = None
        self.last_trusted = None
        self.failures = 0
        self.evidence = deque(maxlen=p.window)
        self.clouds = deque(maxlen=p.window)
        self.search = None
        self.hypotheses = []
        self.verify = []
        self.overflow = False

    def _clear(self):
        self.evidence.clear(); self.clouds.clear()
        self.search = None; self.hypotheses = []; self.verify = []
        self.overflow = False

    def initialize(self, correction, stamp):
        C = np.asarray(correction, float)
        if C.shape != (4, 4) or not np.isfinite(C).all() or not np.isfinite(stamp):
            raise ValueError('invalid initialization')
        if not np.allclose(C, pose_matrix(C[0,3], C[1,3], yaw(C)), atol=1e-5):
            raise ValueError('initialization must be planar')
        self._clear(); self.correction = C.copy()
        self.last_stamp = stamp; self.last_trusted = stamp
        self.failures = 0; self.state = 'SUSPECT'

    def pause(self, enabled):
        self._clear()
        if enabled:
            self.state = 'PAUSED'
        else:
            self.state = 'SUSPECT' if self.correction is not None else 'UNINITIALIZED'
        self.last_stamp = None
        # Resuming never makes the previous correction fresh evidence.

    def expire(self, now):
        if self.state in ('PAUSED', 'UNINITIALIZED'):
            return
        if self.last_stamp is not None and now-self.last_stamp > self.policy.evidence_max_age_s:
            self._clear(); self.state = 'LOST'

    def reject_input(self, now, reason):
        if self.state not in ('PAUSED', 'UNINITIALIZED'):
            self.failures += 1
            # Invalid inputs cannot preserve verification evidence.
            self.evidence.clear(); self.verify = []
            if self.state == 'VERIFYING':
                self._clear(); self.state = 'LOST'
            elif self.state == 'TRACKING':
                self.state = 'SUSPECT'
            self.expire(now)
            if self.failures >= self.policy.lost_after_failures:
                self._clear(); self.state = 'LOST'
        return self._result(reason)

    def _result(self, reason, pose=None, **metrics):
        return pose, dict(state=self.state, accepted=pose is not None,
            reason=reason, failures=self.failures,
            hypotheses=len(self.hypotheses), search_scope='local_only', **metrics)

    def _diverse(self, rows):
        return (rows[-1]['stamp']-rows[0]['stamp'] >= self.policy.min_span_s
                and np.ptp([r['pitch'] for r in rows]) >= self.policy.pitch_span_rad)

    def _consensus(self, odom):
        p = self.policy; rows = list(self.evidence)
        if len(rows) < p.support:
            return None
        # Require the current candidate itself to belong to the consensus.
        best = []
        for centre in rows:
            group = [r for r in rows if all((a <= b) for a,b in zip(
                distance(r['correction'], centre['correction'], odom),
                (p.radius_m,p.radius_yaw_rad)))]
            if any(r is rows[-1] for r in group) and len(group)>len(best):
                best = group
        if len(best)<p.support or not self._diverse(best):
            return None
        # Medoid: choose an observed correction, never average opposing poses.
        return min(best, key=lambda a: sum(distance(a['correction'], b['correction'], odom)[0]
                                           + distance(a['correction'], b['correction'], odom)[1] for b in best))['correction']

    def _start_search(self, odom):
        rows = list(self.clouds)
        if len(rows) < self.policy.window or not self._diverse(rows):
            return False
        # Common frame is the latest cloud's base frame, using stamped odometry.
        inv = np.linalg.inv(odom)
        q = np.concatenate([transform(r['points'], inv@r['odom']) for r in rows])
        # Voxel representatives bound density before the matcher's point cap.
        _, ids = np.unique(np.floor(q/.07).astype(np.int64),axis=0,return_index=True)
        q = q[np.sort(ids)]
        if len(q)>2000:q=q[np.linspace(0,len(q)-1,2000).astype(int)]
        initial = self.correction @ odom; seeds=[]
        # Centre first, followed by 26 neighbors. No unbounded global fallback.
        offsets=[(x,y,a) for x in (-1,0,1) for y in (-1,0,1) for a in (-1,0,1)]
        offsets.sort(key=lambda v:sum(abs(x) for x in v))
        for x,y,a in offsets:
            seeds.append(pose_matrix(initial[0,3]+x*self.policy.search_xy_m,
                initial[1,3]+y*self.policy.search_xy_m,
                yaw(initial)+a*self.policy.search_yaw_rad))
        self.search = dict(points=q, odom=odom.copy(), seeds=deque(seeds))
        self.hypotheses=[]; self.verify=[]; self.overflow=False; self.state='SEARCHING'
        return True

    def _search_step(self):
        search=self.search
        T,r=self.recovery_matcher.match(search['points'],search['seeds'].popleft())
        if r['accepted']:
            C=T@np.linalg.inv(search['odom'])
            if not any(distance(C,h,search['odom'])[0]<.15 and
                       distance(C,h,search['odom'])[1]<np.deg2rad(5) for h in self.hypotheses):
                if len(self.hypotheses)<self.policy.max_hypotheses:self.hypotheses.append(C)
                else:self.overflow=True
        if not search['seeds']:
            self.search=None
            if self.overflow or not self.hypotheses:
                self.state='LOST'; self.clouds.clear()
                return self._result('search_ambiguous' if self.overflow else 'search_no_candidate')
            self.state='VERIFYING'; self.verify=[]
        return self._result('search_step', search_match=r)

    def _score(self, points, pose):
        L=self.matcher.limits
        if len(points)<L.min_points:return None
        if len(points)>2000:points=points[np.linspace(0,len(points)-1,2000).astype(int)]
        moved=transform(points,pose)
        d,idx=self.matcher.tree.query(moved,distance_upper_bound=L.inlier_m)
        ok=np.isfinite(d);overlap=float(ok.mean())
        if ok.sum()<L.min_points or overlap<L.min_overlap:return None
        n=self.matcher.normals[idx[ok]];rel=moved[ok]-pose[:3,3]
        J=np.column_stack((-rel[:,1]*n[:,0]+rel[:,0]*n[:,1],n[:,0],n[:,1]))
        H=J.T@J;diag=np.sqrt(np.maximum(np.diag(H),1e-20))
        cond=np.linalg.cond(H/diag[:,None]/diag[None,:])
        if not np.isfinite(cond) or cond>L.max_condition or np.min(np.diag(H))<1e-8:return None
        plane=float(np.sqrt(np.mean(np.sum((moved[ok]-self.matcher.points[idx[ok]])*n,axis=1)**2)))
        nn=float(np.sqrt(np.mean(d[ok]**2)))
        if plane>L.max_plane_rmse_m or nn>L.max_nn_rmse_m:return None
        return overlap-nn/L.inlier_m

    def _verify_step(self, row):
        scores=[self._score(row['points'],h@row['odom']) for h in self.hypotheses]
        self.verify.append(dict(stamp=row['stamp'],pitch=row['pitch'],scores=scores))
        # Every promoted hypothesis must pass all of the latest verification
        # frames. Inconsistent alternatives retain a low score, not a pose vote.
        recent=self.verify[-self.policy.verification_frames:]
        if len(recent)==self.policy.verification_frames and self._diverse(recent):
            score=np.array([[v if v is not None else -1. for v in r['scores']] for r in recent])
            means=score.mean(axis=0);order=np.argsort(means)[::-1];winner=order[0]
            unique=len(order)==1 or means[winner]-means[order[1]]>=self.policy.score_margin
            if np.all(score[:,winner]>-1.) and unique:
                self.correction=self.hypotheses[winner].copy()
                self.last_trusted=row['stamp'];self.failures=0;self.state='TRACKING'
                self.evidence.clear();self.hypotheses=[];self.verify=[];self.clouds.clear()
                return self._result('reacquired',self.correction@row['odom'])
        if len(self.verify)>=self.policy.verification_limit:
            self._clear();self.state='LOST'
            return self._result('verification_ambiguous_or_failed')
        return self._result('verifying')

    def process(self, points, odom, stamp, pitch, now=None):
        now=stamp if now is None else now
        if not np.isfinite([stamp,now,pitch]).all():
            return self.reject_input(self.last_stamp or 0.,'invalid_time_or_pitch')
        if self.state=='PAUSED':return self._result('intentional_pause')
        if self.correction is None:return self._result('awaiting_initial_pose')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return self.reject_input(now,'out_of_order')
        if not 0<=now-stamp<=.5:return self.reject_input(now,'stale_input')
        if self.last_stamp is not None and stamp-self.last_stamp>self.policy.evidence_max_age_s:
            self._clear();self.state='LOST'
        self.last_stamp=stamp
        points=np.asarray(points,float);odom=np.asarray(odom,float)
        if points.ndim!=2 or points.shape[1]!=3 or odom.shape!=(4,4) or not np.isfinite(odom).all():
            return self.reject_input(now,'invalid_input')
        if not np.allclose(odom,pose_matrix(odom[0,3],odom[1,3],yaw(odom)),atol=1e-5):
            return self.reject_input(now,'nonplanar_odometry')
        points=points[np.isfinite(points).all(axis=1)]
        ranges=np.linalg.norm(points,axis=1);points=points[(ranges>.15)&(ranges<6.)]
        if len(points)<self.matcher.limits.min_points:return self.reject_input(now,'insufficient_points')
        row=dict(points=points,odom=odom,stamp=stamp,pitch=pitch)
        self.clouds.append(row)
        if self.state=='SEARCHING':return self._search_step()
        if self.state=='VERIFYING':return self._verify_step(row)
        if self.state=='LOST':
            self._start_search(odom)
            return self._result('collecting_recovery_geometry')
        T,r=self.matcher.match(points,self.correction@odom)
        while self.evidence and stamp-self.evidence[0]['stamp']>self.policy.evidence_max_age_s:self.evidence.popleft()
        if r['accepted']:
            self.evidence.append(dict(stamp=stamp,pitch=pitch,correction=T@np.linalg.inv(odom)))
            agreed=self._consensus(odom)
            if agreed is not None:
                self.correction=agreed.copy();self.last_trusted=stamp
                self.failures=0;self.state='TRACKING'
                return self._result('temporal_consensus',agreed@odom,match=r)
            # Count persistent inconsistency too, not just ICP rejection.
            self.failures+=1
        else:
            self.failures+=1
        self.state='SUSPECT'
        if self.failures>=self.policy.lost_after_failures and stamp-self.last_trusted>=self.policy.lost_after_s:
            self.state='LOST'
            # Search begins here, but its first fit waits for the next callback.
            self._start_search(odom)
        return self._result('waiting_for_consensus' if r['accepted'] else 'match_rejected',match=r)
