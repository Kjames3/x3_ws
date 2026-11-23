#!/usr/bin/env python3
"""Capture synced left/right/RGB views of a ChArUco board shown on a TV (depthai v3).

Auto-saves a view when both mono cameras see enough corners and the board has
moved since the last save, so one person can reposition the robot/camera alone.

  python3 oak_tv_charuco_capture.py --device 14442C103181D7D600 --square-cm 7.5 --out DIR
Board: 11x8 squares, DICT_4X4_1000, marker = 0.75 square (Luxonis default).
"""
import argparse, os, time
import cv2, numpy as np, depthai as dai

ap = argparse.ArgumentParser()
ap.add_argument('--device', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--square-cm', type=float, required=True)
ap.add_argument('--target', type=int, default=40)
ap.add_argument('--min-corners', type=int, default=25)
ap.add_argument('--min-move-px', type=float, default=60.0)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
BOARD = cv2.aruco.CharucoBoard((11, 8), a.square_cm, a.square_cm * 0.75, DICT)
DET = cv2.aruco.CharucoDetector(BOARD)

def corners(gray):
    c, ids, _, _ = DET.detectBoard(gray)
    return (c, ids) if ids is not None else (None, None)

pipe = dai.Pipeline(dai.Device(dai.DeviceInfo(a.device)))
qs = {}
for name, sock, size, typ in (('left', dai.CameraBoardSocket.CAM_B, (1280, 800), dai.ImgFrame.Type.GRAY8),
                              ('right', dai.CameraBoardSocket.CAM_C, (1280, 800), dai.ImgFrame.Type.GRAY8),
                              ('rgb', dai.CameraBoardSocket.CAM_A, (1920, 1080), dai.ImgFrame.Type.NV12)):
    cam = pipe.create(dai.node.Camera).build(sock)
    if name == 'rgb':
        cam.initialControl.setAutoExposureCompensation(-9)  # TV white blooms the colour sensor
    qs[name] = cam.requestOutput(size, typ, fps=10).createOutputQueue(maxSize=2, blocking=False)
pipe.start()

saved, last_c, t_last = 0, None, 0.0
print(f'capturing to {a.out}; target {a.target} views', flush=True)
while pipe.isRunning() and saved < a.target:
    f = {k: q.get() for k, q in qs.items()}
    ts = {k: v.getTimestamp().total_seconds() for k, v in f.items()}
    if abs(ts['left'] - ts['right']) > 0.005:
        continue
    L, R = f['left'].getCvFrame(), f['right'].getCvFrame()
    cl, il = corners(L); cr, ir = corners(R)
    nl = 0 if il is None else len(il); nr = 0 if ir is None else len(ir)
    rgbf = f['rgb'].getCvFrame(); rgb = cv2.cvtColor(rgbf, cv2.COLOR_BGR2GRAY) if rgbf.ndim == 3 else rgbf
    _, ib = corners(rgb); nb = 0 if ib is None else len(ib)
    now = time.time()
    if now - t_last > 2.0:
        print(f'  seen left {nl:2d} right {nr:2d} rgb {nb:2d} corners   saved {saved}', flush=True); t_last = now
    if nl < a.min_corners or nr < a.min_corners or nb < a.min_corners:
        continue
    c = cl.reshape(-1, 2).mean(0)
    if last_c is not None and np.linalg.norm(c - last_c) < a.min_move_px:
        continue
    for k, img in (('left', L), ('right', R), ('rgb', rgb)):
        cv2.imwrite(os.path.join(a.out, f'{saved:03d}_{k}.png'), img)
    saved += 1; last_c = c
    print(f'SAVED view {saved}/{a.target} (left {nl}, right {nr} corners) - move the robot', flush=True)
pipe.stop()
print('done', flush=True)
