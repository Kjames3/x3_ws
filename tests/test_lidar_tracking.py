"""State/evidence contracts with controlled candidate streams; numerical ICP
recovery is covered separately by integration and recorded-data evaluation.
"""
import sys
from pathlib import Path
from dataclasses import replace
from collections import deque
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Limits,pose_matrix
from yahboomcar_bringup.lidar_tracking import Tracker,Policy


class ControlledMatcher:
    limits=Limits()
    def __init__(self,corrections):self.corrections=deque(corrections)
    def match(self,points,initial):
        C=self.corrections.popleft() if self.corrections else None
        return (initial if C is None else C),dict(accepted=C is not None,reasons=[] if C is not None else ['prior_disagreement'])


POINTS=np.tile([1.,1.,1.],(200,1))

def tracker(corrections,**policy):
    t=Tracker(ControlledMatcher(corrections),replace(Policy(),**policy));t.initialize(pose_matrix(),0.);return t


def feed(t,k,odom=None,pitch=None):
    return t.process(POINTS,pose_matrix() if odom is None else odom,.14*k,.06*k if pitch is None else pitch)


def test_no_single_match_is_trusted_and_consensus_uses_different_angles():
    t=tracker([pose_matrix(.03)]*8)
    for k in range(1,4):assert feed(t,k)[0] is None
    pose,r=feed(t,4);assert r['state']=='TRACKING' and pose is not None
    t=tracker([pose_matrix(.03)]*8)
    for k in range(1,9):assert feed(t,k,pitch=0.)[0] is None


def test_outlier_cannot_vote_itself_into_trusted_pose():
    t=tracker([pose_matrix(.01)]*4+[pose_matrix(.25)])
    for k in range(1,5):feed(t,k)
    before=t.correction.copy();pose,r=feed(t,5)
    assert pose is None and r['state']=='SUSPECT'
    np.testing.assert_array_equal(t.correction,before)


def test_comparison_compensates_robot_motion():
    # Matches move with odometry; correction stays fixed, so consensus must pass.
    C=pose_matrix(.02,-.01,.01);odoms=[pose_matrix(.08*k,0,.03*k) for k in range(1,5)]
    t=tracker([C@o for o in odoms])
    for k,o in enumerate(odoms,1):pose,r=feed(t,k,odom=o)
    assert r['accepted'];np.testing.assert_allclose(t.correction,C,atol=1e-10)


def test_pause_and_unexpected_loss_are_distinct():
    t=tracker([pose_matrix()]*10);t.pause(True);t.expire(10.)
    assert t.state=='PAUSED' and feed(t,1)[0] is None
    t.pause(False);assert t.state=='SUSPECT'
    feed(t,1);t.expire(3.);assert t.state=='LOST' and not t.evidence


def test_gap_clears_old_evidence_and_restarts_recovery():
    t=tracker([pose_matrix()]*10)
    for k in range(1,4):feed(t,k)
    pose,r=t.process(POINTS,pose_matrix(),3.,.3)
    assert pose is None and r['state']=='LOST' and not t.evidence


def test_rejections_enter_bounded_search_without_changing_trust():
    t=tracker([None]*30)
    for k in range(1,13):feed(t,k)
    assert t.state=='SEARCHING' and len(t.search['seeds'])<=27
    np.testing.assert_array_equal(t.correction,np.eye(4))


def prepare_verification(t,hypotheses,scores):
    t.state='VERIFYING';t.hypotheses=hypotheses
    values=iter(scores);t._score=lambda p,T: next(values)


def test_equal_hypotheses_stay_ambiguous_and_never_publish():
    t=tracker([]);prepare_verification(t,[pose_matrix(.4),pose_matrix(-.4)],[.5,.5]*12)
    for k in range(1,13):assert feed(t,k)[0] is None
    assert t.state=='LOST';np.testing.assert_array_equal(t.correction,np.eye(4))


def test_recovery_requires_later_frames_and_unique_support():
    t=tracker([]);prepare_verification(t,[pose_matrix(.4),pose_matrix(-.4)],[.7,.4]*5)
    for k in range(1,5):assert feed(t,k)[0] is None
    pose,r=feed(t,5)
    assert r['reason']=='reacquired' and r['accepted']
    np.testing.assert_allclose(pose,pose_matrix(.4))


def test_duplicate_and_stale_frames_never_count_as_votes():
    t=tracker([pose_matrix()]*10)
    feed(t,1);p,r=feed(t,1)
    assert p is None and r['reason']=='out_of_order' and not t.evidence
    p,r=t.process(POINTS,pose_matrix(),.28,.2,now=1.)
    assert p is None and r['reason']=='stale_input'


def test_search_uses_frozen_geometry_and_separate_verification():
    t=tracker([None]*12)
    for k in range(1,13):feed(t,k)
    # A fake converged candidate from the frozen accumulation.
    t.search['seeds']=deque([pose_matrix()])
    t.recovery_matcher.corrections=deque([pose_matrix(.4)])
    p,r=feed(t,13)
    assert p is None and t.state=='VERIFYING' and not t.verify
    np.testing.assert_array_equal(t.correction,np.eye(4))
