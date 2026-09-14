import sys
from pathlib import Path
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Matcher,Limits,component,pose_matrix,transform


def room():
    rng=np.random.default_rng(12)
    a=rng.uniform(-2,2,(1200,3));a[:400,0]=2;a[400:800,1]=2;a[800:,2]=0
    n=np.zeros_like(a);n[:400,0]=1;n[400:800,1]=1;n[800:,2]=1
    return a,n


def test_recovers_known_planar_transform():
    p,n=room();truth=pose_matrix(.12,-.09,.03)
    query=transform(p,np.linalg.inv(truth))
    T,r=Matcher(p,n,replace(Limits(),max_runtime_s=3.)).match(query,pose_matrix())
    assert r['accepted'],r
    np.testing.assert_allclose(T,truth,atol=.002)


def test_single_wall_rejected_as_degenerate():
    p,n=room();_,r=Matcher(p[:400],n[:400]).match(p[:400],pose_matrix())
    assert not r['accepted'] and 'degenerate_geometry' in r['reasons']


def test_no_correspondences_and_invalid_points_rejected():
    p,n=room();m=Matcher(p,n)
    for q,prior in [(p,pose_matrix(50,50)),(np.full((200,3),np.nan),pose_matrix()),(np.empty((0,3)),pose_matrix())]:
        _,r=m.match(q,prior);assert not r['accepted']


def test_good_fit_cannot_override_large_prior_correction():
    p,n=room();truth=pose_matrix(.12,-.09,.03);query=transform(p,np.linalg.inv(truth))
    _,r=Matcher(p,n,replace(Limits(),max_correction_m=.05,max_runtime_s=3.)).match(query,pose_matrix())
    assert not r['accepted'] and 'prior_disagreement' in r['reasons']


def test_runtime_budget_withholds_result():
    p,n=room();_,r=Matcher(p,n,replace(Limits(),max_runtime_s=0.)).match(p,pose_matrix())
    assert r['reasons']==['runtime_budget']


def test_connected_component_excludes_unanchored_stations():
    edges=[{'i':0,'j':1},{'i':1,'j':2},{'i':0,'j':4},{'i':5,'j':6}]
    assert component(edges)==[0,1,2,4]
