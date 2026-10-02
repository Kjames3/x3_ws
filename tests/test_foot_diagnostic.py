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
