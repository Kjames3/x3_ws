import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from c1_camera_geometry import rgb_1080_preview_crop, rgb_1080_preview_intrinsics


def test_projection_matches_crop_bin_then_centre_crop():
    k=np.array([[3000.,0.,2104.],[0.,3000.,1560.],[0.,0.,1.]])
    out=rgb_1080_preview_intrinsics(k,(4208,3120),(480,640),'IMX214')
    ray=np.array([.1,-.2,1.]);native=k@ray
    # 1080P bins the centred 3840x2160; the 3:4 preview keeps the middle 810x1080.
    isp=(native[:2]-[184,480])/2
    expected=(isp-[(1920-810)/2,0])*(640/1080)
    np.testing.assert_allclose((out@ray)[:2],expected)
    np.testing.assert_allclose(out[:2,2],[240,320])


def test_imx378_preview_is_uniformly_scaled():
    # OAK-D Pro W: EEPROM at 3840x2160, fx ~= fy ~= 2283.
    k=np.array([[2283.1,0.,1945.],[0.,2283.1,1085.],[0.,0.,1.]])
    out=rgb_1080_preview_intrinsics(k,(3840,2160),(480,640),'IMX378')
    s=640/2160
    np.testing.assert_allclose([out[0,0],out[1,1]],[2283.1*s]*2)   # ~676, not ~285
    np.testing.assert_allclose(out[:2,2],[(1945-1110)*s,1085*s])
    np.testing.assert_allclose(rgb_1080_preview_crop('IMX378',(3840,2160),(480,640)),
                               (1110,0,1620,2160))


def test_landscape_preview_crops_rows():
    x,y,w,h=rgb_1080_preview_crop('IMX378',(3840,2160),(640,300))
    assert (x,w)==(0,3840) and np.isclose(w/h,640/300) and np.isclose(y,(2160-h)/2)


def test_unqualified_sensor_or_calibration_refused():
    for size,name in [((4056,3040),'IMX378'),((1920,1080),'IMX214'),((3840,2160),'OV9782')]:
        with pytest.raises(ValueError):
            rgb_1080_preview_intrinsics(np.eye(3),size,(480,640),name)
