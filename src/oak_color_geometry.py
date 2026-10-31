"""Bounded display-only OAK geometry; no ROS or device access."""
import numpy as np
import cv2

def prepare_geometry(rgb, depth, support, correction, edge_margin=12):
    if rgb.shape != (*depth.shape, 3) or support.shape != depth.shape:
        raise ValueError('RGB/depth/support shapes disagree')
    if edge_margin < 0:
        raise ValueError('invalid budget or margin')
    z = depth.astype(np.float32) * .001
    z /= 1 + correction * z
    valid = support.astype(bool) & np.isfinite(z) & (z >= .4) & (z <= 3.)
    # Reject depth jumps and invalid neighborhoods. Margin is an experimental
    # exclusion band, not a fitted pixel shift or a calibration correction.
    high = cv2.dilate(np.where(valid, z, 0), np.ones((3, 3), np.uint8))
    low = cv2.erode(np.where(valid, z, 10), np.ones((3, 3), np.uint8))
    bad = (~valid) | ((high - low) > .08)
    size = 2 * edge_margin + 1
    safe = valid & ~cv2.dilate(bad.astype(np.uint8), np.ones((size, size), np.uint8)).astype(bool)
    return z, valid, safe


def balanced_pixels(valid, budget):
    """One representative per image tile; tile count never exceeds budget.

    Unlike row-major linspace over valid pixels, blank regions do not distort
    the spacing elsewhere. Sparse tiles use the valid pixel nearest tile center.
    """
    if budget < 1:
        raise ValueError('budget must be positive')
    h, w = valid.shape
    step = max(1, int(np.ceil(np.sqrt(h*w/budget))))
    while int(np.ceil(h/step))*int(np.ceil(w/step)) > budget:
        step += 1
    rows, cols = int(np.ceil(h/step)), int(np.ceil(w/step))
    ty,tx = np.indices((rows,cols))
    y = np.minimum(ty.ravel()*step+step//2,h-1)
    x = np.minimum(tx.ravel()*step+step//2,w-1)
    good = valid[y,x]
    missing = np.flatnonzero(~good)
    if len(missing):
        dy,dx = np.indices((step,step))
        yy=ty.ravel()[missing,None]*step+dy.ravel()
        xx=tx.ravel()[missing,None]*step+dx.ravel()
        inside=(yy<h)&(xx<w)
        candidates=inside&valid[np.minimum(yy,h-1),np.minimum(xx,w-1)]
        cost=((dy-(step-1)/2)**2+(dx-(step-1)/2)**2).ravel()
        choice=np.argmin(np.where(candidates,cost,np.inf),axis=1)
        y[missing]=yy[np.arange(len(missing)),choice]
        x[missing]=xx[np.arange(len(missing)),choice]
        good[missing]=candidates.any(axis=1)
    return y[good],x[good]



def refine(rgb, depth, support, k, correction, budget=6000, margin=12):
    # Reuse the earlier conservative mask as COLOR confidence only. It is not
    # a geometry correctness guarantee and does not fix stereo edge fattening.
    z, valid, confident = prepare_geometry(rgb, depth, support, correction, margin)
    y,x=balanced_pixels(valid,budget)
    zz=z[y,x]
    xyz=np.column_stack(((x-k[0,2])*zz/k[0,0],(y-k[1,2])*zz/k[1,1],zz)).astype('<f4')
    color_valid=confident[y,x]
    colors=np.full((len(x),3),160,dtype=np.uint8)
    colors[color_valid]=rgb[y[color_valid],x[color_valid],::-1]
    return xyz,colors,color_valid,np.column_stack((y,x))
