import numpy as np
from oak_color_cloud_refine import balanced_pixels, refine


def test_sampling_is_bounded_and_covers_separated_regions():
    mask=np.zeros((40,80),bool);mask[3:8,3:8]=1;mask[32:38,72:78]=1
    y,x=balanced_pixels(mask,100)
    assert 0<len(x)<=100 and mask[y,x].all()
    assert (x<10).any() and (x>70).any()
    assert len(set(zip(y,x)))==len(x)
    assert len(balanced_pixels(np.zeros((4,4),bool),10)[0])==0


def test_uncertain_geometry_survives_without_borrowing_rgb():
    depth=np.full((40,40),1000,np.uint16);depth[:,20:]=2000
    rgb=np.zeros((40,40,3),np.uint8);rgb[:]=[0,0,255]
    k=np.array([[100,0,20],[0,100,20],[0,0,1]])
    xyz,colors,confident,pixels=refine(rgb,depth,np.ones(depth.shape,bool),k,.0821,budget=1600,margin=2)
    assert len(xyz)==1600
    assert confident.any() and (~confident).any()
    assert np.all(colors[confident]==[255,0,0])
    assert np.all(colors[~confident]==160)
    assert np.isclose(xyz[:,2].min(),1/1.0821)
    assert np.isclose(xyz[:,2].max(),2/1.1642)
