import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from c1_alignment import boundary_interval, measure, summarize


def test_invalid_depth_is_uncertainty_not_a_matching_edge():
    d=np.full(100,1800,dtype=np.uint16);d[45:55]=0;d[55:]=1000
    assert boundary_interval(d,50,1000,'left')==(44.,55.)
    assert boundary_interval(np.zeros(100),50,1000,'left') is None


def test_displaced_target_edge_and_support_are_reported():
    rgb=np.zeros((40,100,3),dtype=np.uint8);rgb[:,25:75]=255
    depth=np.full((40,100),1800,dtype=np.uint16);depth[:,29:79]=1000
    rows=measure(rgb,depth,np.ones((40,100),bool),(25,5,75,35),1000)
    assert all(r['signed_midpoint_error_px']==4 for r in rows)
    assert summarize(rows)['local_edge_gate_pass']
    assert not summarize(measure(rgb,depth,np.zeros((40,100),bool),(25,5,75,35),1000))['local_edge_gate_pass']
    depth[:]=1800;depth[:,35:85]=1000
    assert not summarize(measure(rgb,depth,np.ones((40,100),bool),(25,5,75,35),1000))['local_edge_gate_pass']


def test_both_axes_recover_known_registration_offset():
    from c1_alignment import measure_rectangle
    rgb=np.zeros((80,100,3),dtype=np.uint8);rgb[20:60,25:75]=255
    d=np.full((80,100),1800,dtype=np.uint16);d[18:58,29:79]=1000
    rows=measure_rectangle(rgb,d,np.ones(d.shape,bool),(25,20,75,60),1000)
    for axis,expected in [('x',4),('y',-2)]:
        measured=[r for r in rows if r['axis']==axis and r['status']=='measured']
        assert measured and all(r['signed_midpoint_error_px']==expected for r in measured)
    assert {r['side'] for r in rows}=={'left','right','top','bottom'}
