import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from c1_subpixel_report import range_evidence,separate_edge_evidence


def test_invalid_depth_not_counted_as_zero_range():
    r=range_evidence([np.array([[0,1510],[1510,0]]),np.zeros((2,2)),np.full((2,2),1490)],1500)
    assert r['frame_median_error_mm']['mean']==0
    assert r['frame_median_mm']['n']==2
    assert r['frames'][1]['median_mm'] is None
    assert r['frames'][0]['valid_fraction']==.5


def test_uncertain_correspondence_cannot_become_a_pass():
    rows=[dict(status='measured',signed_midpoint_error_px=0,
               absolute_error_lower_px=0,absolute_error_upper_px=20,uncertainty_width_px=40)]
    rows.append(dict(status='no_unique_depth_transition'))
    r=separate_edge_evidence(rows)
    assert r['measured_fraction']==.5 and not r['local_edge_gate_pass']
    assert r['absolute_midpoint_error_px_measured_only']['median']==0
    assert r['narrow_interval_diagnostic']['samples']==0
