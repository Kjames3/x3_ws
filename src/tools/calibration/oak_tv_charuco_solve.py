#!/usr/bin/env python3
"""Solve OAK left/right/RGB calibration from oak_tv_charuco_capture.py views.

Rational 8-coef model (what the Luxonis factory calibration uses). Intrinsics
per camera, then stereo extrinsics left->right and rgb->right with the
intrinsics fixed. Prints a comparison against an EEPROM JSON backup and writes
the result to --out-json (nothing is flashed here).
"""
import argparse, glob, json, os
import cv2, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('views')
ap.add_argument('--square-cm', type=float, required=True)
ap.add_argument('--backup', required=True)
ap.add_argument('--out-json', required=True)
a = ap.parse_args()

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
BOARD = cv2.aruco.CharucoBoard((11, 8), a.square_cm, a.square_cm * 0.75, DICT)
DET = cv2.aruco.CharucoDetector(BOARD)
OBJ = BOARD.getChessboardCorners()
FLAGS = cv2.CALIB_RATIONAL_MODEL
CRIT = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)

n = len(glob.glob(os.path.join(a.views, '*_left.png')))
det = {k: [] for k in ('left', 'right', 'rgb')}
size = {}
for i in range(n):
    for k in det:
        g = cv2.imread(os.path.join(a.views, f'{i:03d}_{k}.png'), 0)
        size[k] = g.shape[::-1]
        c, ids, _, _ = DET.detectBoard(g)
        det[k].append(None if ids is None or len(ids) < 8 else (c.reshape(-1, 2), ids.ravel()))

def mono(k):
    op, ip = [], []
    for d in det[k]:
        if d: op.append(OBJ[d[1]].astype(np.float32)); ip.append(d[0].astype(np.float32))
    rms, K, D, rv, tv, _, _, per = cv2.calibrateCameraExtended(op, ip, size[k], None, None, flags=FLAGS, criteria=CRIT)
    return rms, K, D, per.ravel()

def pair(ka, kb, Ka, Da, Kb, Db):
    op, pa, pb = [], [], []
    for da, db in zip(det[ka], det[kb]):
        if not da or not db: continue
        common = np.intersect1d(da[1], db[1])
        if len(common) < 8: continue
        ia = [np.where(da[1] == c)[0][0] for c in common]; ib = [np.where(db[1] == c)[0][0] for c in common]
        op.append(OBJ[common].astype(np.float32)); pa.append(da[0][ia].astype(np.float32)); pb.append(db[0][ib].astype(np.float32))
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(op, pa, pb, Ka, Da, Kb, Db, size[ka],
                                                      flags=cv2.CALIB_FIX_INTRINSIC | FLAGS, criteria=CRIT)
    return rms, R, T.ravel(), len(op)

bk = {int(s): v for s, v in json.load(open(a.backup))['cameraData']}
SOCK = {'rgb': 0, 'left': 1, 'right': 2}
res = {}
for k in ('left', 'right', 'rgb'):
    rms, K, D, per = mono(k)
    res[k] = dict(K=K, D=D)
    old = bk[SOCK[k]]; s = size[k][0] / old['width']
    print(f'{k:5s} views {sum(d is not None for d in det[k]):2d} rms {rms:.3f}px  worst view {per.max():.2f}px  '
          f'fx {K[0,0]:.1f} (old {old["intrinsicMatrix"][0][0]*s:.1f})  cx {K[0,2]:.1f} (old {old["intrinsicMatrix"][0][2]*s:.1f})  '
          f'cy {K[1,2]:.1f} (old {old["intrinsicMatrix"][1][2]*s:.1f})')
rms, R, T, m = pair('left', 'right', res['left']['K'], res['left']['D'], res['right']['K'], res['right']['D'])
ot = bk[1]['extrinsics']['translation']
print(f'left->right  pairs {m} rms {rms:.3f}px  T {np.round(T,3)} cm  baseline {np.linalg.norm(T):.3f} cm '
      f'(old {[round(ot[x],3) for x in "xyz"]})  rot {np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])):.3f} deg')
rms2, R2, T2, m2 = pair('rgb', 'right', res['rgb']['K'], res['rgb']['D'], res['right']['K'], res['right']['D'])
ot = bk[0]['extrinsics']['translation']
print(f'rgb->right   pairs {m2} rms {rms2:.3f}px  T {np.round(T2,3)} cm (old {[round(ot[x],3) for x in "xyz"]} to sock {bk[0]["extrinsics"]["toCameraSocket"]})')
json.dump({k: {'K': res[k]['K'].tolist(), 'D': res[k]['D'].ravel().tolist(), 'size': size[k]} for k in res} |
          {'left_to_right': {'R': R.tolist(), 'T_cm': T.tolist(), 'rms': rms},
           'rgb_to_right': {'R': R2.tolist(), 'T_cm': T2.tolist(), 'rms': rms2}}, open(a.out_json, 'w'), indent=1)
