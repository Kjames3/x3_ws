import ast
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from foot_diagnostic import (measure_feet, FootTracker, FootDiagnostic, shadow_cbf)


def scene():
    w,h=480,640
    intr=(400.,400.,240.,320.,w,h)
    yy=np.arange(h)[:,None]
    depth=np.broadcast_to(np.divide(.213*400,yy-320,out=np.zeros((h,1)),where=yy>320),(h,w)).copy().astype('float32')
    k=[[0,0,0] for _ in range(17)]
    k[13]=[240,350,.95];k[15]=[240,440,.95]
    detection={'label':'person','keypoints':k,'keypoints_xyz':[[99,99,99]]*17}
    return depth,intr,detection


def test_depth_foot_not_torso_plane():
    depth,intr,d=scene()
    depth[430:451,230:251]=.5
    feet,rejected=measure_feet([d],depth,intr,(480,640),(.108,.213,0))
    assert len(feet)==1
    assert abs(feet[0]['xy'][0]-.608)<.001
    assert abs(feet[0]['xy'][1])<.001
    assert feet[0]['height_m']>.04
    assert feet[0]['pixels']>=16
    assert rejected['ankle_not_visible']==1


def test_floor_invalid_low_confidence_and_clipped_rejected():
    depth,intr,d=scene()
    assert not measure_feet([d],depth,intr,(480,640),(.108,.213,0))[0]
    depth[:]=np.nan
    assert not measure_feet([d],depth,intr,(480,640),(.108,.213,0))[0]
    depth[:]=.5;d['keypoints'][15][2]=.1
    assert not measure_feet([d],depth,intr,(480,640),(.108,.213,0))[0]
    d['keypoints'][15]=[2,440,.95]
    assert measure_feet([d],depth,intr,(480,640),(.108,.213,0))[1]['foot_crop_clipped']==1


def measurement(x,y=0,side='left'):
    return dict(xy=np.array([x,y]),side=side,radius_m=.07,sigma_m=.025,height_m=.06,confidence=.95,pixels=80,roi=[1,2,3,4])


def test_world_tracking_compensates_robot_translation_and_rotation():
    tracker=FootTracker()
    for i in range(5):
        angle=i*.05;x=i*.02
        # A stationary world foot at [1,0], viewed from a moving/turning robot.
        m=measurement((1-x)*math.cos(angle),-(1-x)*math.sin(angle))
        tracker.update([m],i*.1,dict(x=x,y=0,theta=angle))
    feet=tracker.local(.4,dict(x=.08,y=0,theta=.2))
    assert len(feet)==1 and feet[0]['velocity_ready']
    assert abs(feet[0]['vfwd'])<1e-8 and abs(feet[0]['vleft'])<1e-8
    assert not tracker.local(2,dict(x=0,y=0,theta=0))
    tracker.update([],.5,dict(x=0,y=0,theta=0))
    assert not tracker.tracks


def test_independent_foot_velocity_and_identity():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    for i in range(5):
        tracker.update([measurement(1-i*.02,-.1),measurement(1,.1,'right')],i*.1,pose)
    feet=tracker.local(.4,pose)
    assert feet[0]['vfwd']<-.15
    assert abs(feet[1]['vfwd'])<1e-9
    assert len({f['id'] for f in feet})==2


def foot(x,v=0):
    return dict(fwd=x,left=0,vfwd=v,vleft=0,radius_m=.07,sigma_m=.025,velocity_ready=True)


def test_shadow_cbf_approach_retreat_infeasibility():
    assert shadow_cbf([])['velocity'] is None
    assert abs(shadow_cbf([foot(.6)])['velocity']['fwd'])<1e-8
    result=shadow_cbf([foot(.6,-.2)])
    assert result['status']=='suggestion' and result['velocity']['fwd']<-.04
    assert shadow_cbf([foot(.6,.2)])['velocity']['fwd']==0
    result=shadow_cbf([foot(.2),foot(-.2)])
    assert result['status']=='infeasible' and result['velocity'] is None
    json.dumps(result,allow_nan=False)


def test_capture_pose_interpolation_and_snapshot_expiry():
    diag=FootDiagnostic(None,None,(.108,.213,0))
    diag.poses.extend([(1,dict(x=0,y=0,theta=3.1)),(2,dict(x=1,y=0,theta=-3.1))])
    p=diag._pose_at(1.5)
    assert abs(p['x']-.5)<1e-9 and abs(p['theta']-math.pi)<1e-6
    assert diag._pose_at(.9) is None
    diag.last_update=time.monotonic()-.1
    diag.result=dict(status='ok',capture_age_s=.6,feet=[foot(.6)],shadow=shadow_cbf([foot(.6)]))
    assert diag.snapshot()['status']=='stale' and not diag.snapshot()['feet']


def test_no_control_integration():
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'src/server_x3.py').read_text())
    refs=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Attribute) and isinstance(node.value,ast.Name) and node.value.id=='foot_diagnostic':
            refs.append(node.attr)
    assert set(refs)=={'start','stop','snapshot'}
    module=(root/'src/foot_diagnostic.py').read_text()
    assert 'import rclpy' not in module and 'publish(' not in module


def test_worker_timestamped_pipeline_and_missing_depth():
    depth,intr,d=scene();depth[430:451,230:251]=.5
    class Camera:
        nn_w,nn_h=480,640
        start=time.monotonic()
        missing=False
        def get_detection_observation(self):
            seq=int((time.monotonic()-self.start)*10)
            self.stamp=self.start+seq*.1-.06
            return [d],dict(session_id='synthetic',seq=seq,
                            host_monotonic_estimate_ns=int(self.stamp*1e9),sdk_host_ns=int(self.stamp*1e9))
        def get_depth_near(self,stamp,max_gap_s):
            return None if self.missing else (stamp,depth)
        def get_depth_intrinsics(self):
            return intr
    camera=Camera()
    diag=FootDiagnostic(camera,lambda:dict(x=0,y=0,theta=0),(.108,.213,0))
    diag.start()
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:
            result=diag.snapshot()
            if result.get('feet') and result['feet'][0]['velocity_ready']:
                break
            time.sleep(.03)
        assert result['status']=='ok' and result['feet'][0]['velocity_ready']
        assert result['shadow']['status']=='suggestion'
        timing=result['timing']
        stages=['pose_lookup','depth_lookup','extraction','tracking','shadow_cbf','result_build']
        assert all(timing[s+'_wall_ms']>=0 and timing[s+'_cpu_ms']>=0 for s in stages)
        assert abs(sum(timing[s+'_wall_ms'] for s in stages)-timing['work_wall_ms'])<1e-6
        assert abs(sum(timing[s+'_cpu_ms'] for s in stages)-timing['work_cpu_ms'])<1e-6
        assert timing['ready_to_snapshot_ms']>=0
        assert timing['capture_to_snapshot_ms']>=timing['capture_to_ready_ms']
        assert result['settings']['nn_w']==480
        json.dumps(result,allow_nan=False)
        camera.missing=True
        end=time.monotonic()+1
        while time.monotonic()<end and diag.snapshot()['status']!='unsynchronized_depth':
            time.sleep(.03)
        result=diag.snapshot()
        assert result['status']=='unsynchronized_depth' and not result['feet']
        assert result['shadow']['velocity'] is None
    finally:
        diag.stop()
    assert not diag.thread.is_alive()


def test_shared_depth_rejects_ambiguous_pair_and_recovers():
    depth,intr,d=scene()
    depth[430:451,230:251]=.5
    d['keypoints'][14]=[242,350,.95]
    d['keypoints'][16]=[242,440,.95]
    feet,rejected=measure_feet([d],depth,intr,(480,640),(.108,.213,0))
    assert feet==[] and rejected['ambiguous_shared_depth']==2
    # Separate components can have overlapping ROIs without sharing support.
    depth,intr,d=scene()
    depth[435:446,225:236]=.5
    depth[435:446,245:256]=.5
    d['keypoints'][15]=[230,440,.95]
    d['keypoints'][14]=[250,350,.95]
    d['keypoints'][16]=[250,440,.95]
    feet,rejected=measure_feet([d],depth,intr,(480,640),(.108,.213,0))
    assert len(feet)==2 and not rejected.get('ambiguous_shared_depth')
    assert all('_support_pixels' not in f for f in feet)


def test_shared_depth_does_not_compare_separate_people():
    depth,intr,d=scene();depth[430:451,230:251]=.5
    import copy
    other=copy.deepcopy(d)
    other['keypoints'][16]=other['keypoints'][15]
    other['keypoints'][14]=other['keypoints'][13]
    other['keypoints'][15]=[0,0,0]
    feet,rejected=measure_feet([d,other],depth,intr,(480,640),(.108,.213,0))
    assert len(feet)==2 and not rejected.get('ambiguous_shared_depth')


def test_smoothing_reduces_stationary_noise_without_holding_missing_feet():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    raw=[];filtered=[];speeds=[]
    for i in range(80):
        x=1+(.01 if i%2 else -.01)
        tracker.update([measurement(x)],i*.1,pose)
        if i>10:
            raw.append(x);filtered.append(tracker.tracks[0]['world'][0])
            speeds.append(abs(tracker.tracks[0]['velocity'][0]))
    assert np.std(filtered)<.65*np.std(raw)
    assert np.percentile(speeds,95)<.06
    tracker.update([],8,pose)
    assert tracker.tracks==[]
    tracker.update([measurement(1)],8.1,pose)
    assert tracker.tracks[0]['count']==1
    assert tracker.tracks[0]['smoothing_lag_m']==0


def test_smoothing_motion_response_and_stop_at_variable_rates():
    pose=dict(x=0,y=0,theta=0)
    for dt in [.05,.1,.2]:
        tracker=FootTracker()
        for stamp in np.arange(0,1+dt/2,dt):
            tracker.update([measurement(1-.2*stamp)],stamp,pose)
        t=tracker.tracks[0]
        assert abs(t['velocity'][0]+.2)<.01
        assert abs(t['world'][0]-.8)<.015
        assert t['smoothing_lag_m']>=abs(t['raw_world'][0]-t['world'][0])-1e-10
        for stamp in np.arange(1+dt,1.6+dt/2,dt):
            tracker.update([measurement(.8)],stamp,pose)
        assert abs(tracker.tracks[0]['velocity'][0])<.01


def test_identity_retention_has_no_stale_output_and_resets_velocity():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    for stamp in [0,.1,.2]:tracker.update([measurement(1)],stamp,pose)
    ident=tracker.tracks[0]['id']
    tracker.update([],.3,pose)
    assert tracker.local(.3,pose)==[] and tracker.dormant[0]['id']==ident
    tracker.update([measurement(1.01)],.4,pose)
    assert tracker.tracks[0]['id']==ident
    assert not tracker.local(.4,pose)[0]['velocity_ready']
    assert np.linalg.norm(tracker.tracks[0]['velocity'])==0
    assert tracker.events[0]['reason']=='reacquired_velocity_reset'
    tracker.update([measurement(1.01)],.5,pose)
    tracker.update([measurement(1.01)],.6,pose)
    assert tracker.local(.6,pose)[0]['velocity_ready']
    tracker.update([],.7,pose)
    tracker.update([measurement(1.01)],1.0,pose)
    assert tracker.tracks[0]['id']!=ident
    assert any(e['reason']=='expired' for e in tracker.events)
    tracker.clear()
    assert not tracker.tracks and not tracker.dormant


def test_tracking_event_reports_speed_and_distance_rejection():
    pose=dict(x=0,y=0,theta=0);tracker=FootTracker()
    tracker.update([measurement(1)],0,pose)
    tracker.update([measurement(1.27)],.08,pose)
    e=tracker.events[0]
    assert e['reason']=='raw_speed_rejected' and e['raw_speed_mps']>3
    assert e['raw_world_xy']==[1.27,0]
    tracker.update([measurement(2)],.16,pose)
    assert tracker.events[0]['reason']=='association_rejected'
    assert tracker.events[0]['candidates'][0]['residual_m']>.3
    json.dumps(tracker.events,allow_nan=False)


def test_replacement_retires_old_identity_before_return_to_old_position():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    tracker.update([measurement(1)],0,pose)
    original=tracker.tracks[0]['id']
    tracker.update([measurement(1.4)],.1,pose)
    replacement=tracker.tracks[0]['id']
    assert replacement!=original and not tracker.dormant
    assert any(e['reason']=='superseded_identity_retired' for e in tracker.events)
    tracker.update([measurement(1)],.2,pose)
    assert tracker.tracks[0]['id']!=original


def test_active_identity_beats_closer_dormant_candidate():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    # Two same-side tracks emulate separate people; one disappears.
    tracker.update([measurement(1),measurement(1.1)],0,pose)
    active,retained=tracker.tracks
    tracker.tracks=[active];tracker.dormant=[retained]
    tracker.update([measurement(1.09)],.1,pose)
    assert tracker.tracks[0]['id']==active['id']
    assert not tracker.dormant
    assert any(e['reason']=='superseded_identity_retired' and e['id']==retained['id']
               for e in tracker.events)


def test_other_side_disappearance_is_still_retained():
    tracker=FootTracker();pose=dict(x=0,y=0,theta=0)
    tracker.update([measurement(1),measurement(1,.2,'right')],0,pose)
    right=tracker.tracks[1]['id']
    tracker.update([measurement(1)],.1,pose)
    assert [t['id'] for t in tracker.dormant]==[right]
    assert len(tracker.local(.1,pose))==1
    tracker.update([measurement(1),measurement(1,.2,'right')],.2,pose)
    assert tracker.tracks[1]['id']==right
    assert not tracker.local(.2,pose)[1]['velocity_ready']


def test_foreground_layer_survives_crop_occupancy_change():
    from foot_diagnostic import select_ankle_component
    for size in [12,18,24]:
        z=np.full((73,73),1.3,dtype=np.float32)
        z[40:40+size,28:28+size]=1.0
        use=select_ankle_component(z,np.ones_like(z,dtype=bool),(36,36),36)
        assert use is not None and np.median(z[use])==1.0


def test_foreground_speckles_and_remote_corner_not_selected():
    from foot_diagnostic import select_ankle_component
    z=np.full((73,73),1.3,dtype=np.float32)
    z[30:33,30:33]=.5
    use=select_ankle_component(z,np.ones_like(z,dtype=bool),(36,36),36)
    assert np.isclose(np.median(z[use]),1.3)
    z[:15,:15]=.5
    use=select_ankle_component(z,np.ones_like(z,dtype=bool),(36,36),36)
    assert np.isclose(np.median(z[use]),1.3)
