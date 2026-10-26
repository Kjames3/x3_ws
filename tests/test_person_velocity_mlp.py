import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import c3_person_tracker as pt
from velocity_estimator import VelocityEstimator
from person_velocity_mlp import PersonVelocityMLP


class DifferenceModel:
    def __call__(self, x):
        return x[:, -2:] / .1


def adapter():
    # Exact derivative model isolates the adapter's frame/cadence contract.
    est=SimpleNamespace(_model=DifferenceModel(), scaler_X_mean=np.zeros(40),
                        scaler_X_inv_scale=np.ones(40), scaler_y_scale=np.ones(2),
                        scaler_y_mean=np.zeros(2),
                        _build_window_features=lambda h: VelocityEstimator._build_window_features(None,h))
    return PersonVelocityMLP(estimator=est)


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[s,0,c],[-c,0,s],[0,-1,0]])


@pytest.mark.parametrize('yaw',[0.,.7,-2.4])
def test_irregular_raw_measurements_recover_world_velocity(yaw):
    net=adapter(); v=np.array([.4,-.2])
    for t in [0,.08,.19,.30,.44,.51,.64,.73,.86,.98,1.09]:
        o=net.update(1,t,np.array([12.,-6.])+v*t,rotation(yaw))
    np.testing.assert_allclose([o['vx'],o['vy']],v,atol=1e-6)


def test_duplicate_and_gap_do_not_manufacture_velocity():
    net=adapter()
    for t in np.arange(12)*.1:
        result=net.update(1,float(t),[1+t*.4,0],rotation(0))
    assert result is not None
    assert net.update(1,1.1,[100,100],rotation(0)) is None
    assert net.update(1,2.,[1.8,0],rotation(0)) is None
    assert len(net.histories[1])==1
    net.retain([])
    assert not net.histories and not net.previous


def test_kalman_reports_exact_raw_measurement_association():
    tr=pt.KalmanTracker()
    for i in range(5):
        z=[(np.array([1.,0.]),np.eye(2)*.01),(np.array([3.,0.]),np.eye(2)*.01)]
        if i%2: z.reverse()
        outs=tr.update(i*.1,z)
        for o in outs:
            assert o['measurement_index'] is not None
            assert abs(z[o['measurement_index']][0][0]-o['x'])<.1
    outs=tr.update(.5,[])
    assert all(o['measurement_index'] is None for o in outs)


def test_uncertainty_defaults_preserved_and_prow_is_configurable():
    old=pt.measurement_cov_camera(3.,.05,.1)
    np.testing.assert_allclose(old,np.eye(2)*(.05**2+.1**2))
    prow=pt.measurement_cov_camera(3.,.05,.1,depth_noise_coef=.009,spread_scale=.5)
    assert prow[0,0]==pytest.approx((.009*9)**2+.05**2)
    assert prow[1,1]==pytest.approx(.05**2+.05**2)
    with pytest.raises(ValueError):
        pt.measurement_cov_camera(2.,depth_noise_coef=-1)


def test_live_measurement_path_honors_covariance_override():
    from c3_live import C3Live
    live=C3Live(None,lambda:None,0.,tracker_config={
        'meas_sigma_floor_m':.01,'depth_noise_coef':.02,'measurement_spread_scale':.5})
    measurements=live._measurements(
        [{'label':'person','bbox':[0,0,40,40]}],np.full((40,40),2.,dtype=np.float32),
        (40,40,20,20,40,40),{'x':0.,'y':0.,'theta':0.})
    assert len(measurements)==1
    np.testing.assert_allclose(measurements[0][1],np.diag([.08**2,.01**2]))


def test_model_pair_must_be_explicit():
    with pytest.raises(ValueError,match='model_path'):
        PersonVelocityMLP()
