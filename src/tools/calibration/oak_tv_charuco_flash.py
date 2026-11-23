#!/usr/bin/env python3
"""Flash an oak_tv_charuco_solve.py result into an OAK's user calibration (depthai v3).

Starts from the device's current EEPROM (backup JSON) so board/product fields
stay, replaces left/right/RGB intrinsics + distortion, left->right and
right->RGB extrinsics and the stereo rectification. RGB is solved at a 2x
downscale of the stored 3840x2160 and scaled up. Factory area is untouched.
Restore with:  --restore BACKUP.json
"""
import argparse, json
import cv2, numpy as np, depthai as dai

ap = argparse.ArgumentParser()
ap.add_argument('--device', required=True)
ap.add_argument('--backup', required=True)
ap.add_argument('--solve')
ap.add_argument('--restore', action='store_true')
ap.add_argument('--stereo-only', action='store_true', help='keep factory lens + RGB; replace only left->right extrinsics and rectification')
ap.add_argument('--dry-run', action='store_true')
a = ap.parse_args()

S = dai.CameraBoardSocket
cal = dai.CalibrationHandler(a.backup)
if not a.restore:
    s = json.load(open(a.solve))
    def d14(d): return list(d) + [0.0] * (14 - len(d))
    for k, sock in (() if a.stereo_only else (('left', S.CAM_B), ('right', S.CAM_C))):
        cal.setCameraIntrinsics(sock, s[k]['K'], 1280, 800)
        cal.setDistortionCoefficients(sock, d14(s[k]['D']))
    if not a.stereo_only:
        Kc = np.array(s['rgb']['K']); scale = 3840 / s['rgb']['size'][0]
        Kc[:2] *= scale
        cal.setCameraIntrinsics(S.CAM_A, Kc.tolist(), 3840, 2160)
        cal.setDistortionCoefficients(S.CAM_A, d14(s['rgb']['D']))
    R = np.array(s['left_to_right']['R']); T = np.array(s['left_to_right']['T_cm'])
    cal.setCameraExtrinsics(S.CAM_B, S.CAM_C, R.tolist(), T.tolist(), [-7.5, 0.0, 0.0])
    R2 = np.array(s['rgb_to_right']['R']); T2 = np.array(s['rgb_to_right']['T_cm'])
    if not a.stereo_only: cal.setCameraExtrinsics(S.CAM_C, S.CAM_A, R2.T.tolist(), (-R2.T @ T2).tolist(), [3.75, 0.0, 0.0])
    Kl, Dl = np.array(s['left']['K']), np.array(s['left']['D']); Kr, Dr = np.array(s['right']['K']), np.array(s['right']['D'])
    R1, Rr, *_ = cv2.stereoRectify(Kl, Dl, Kr, Dr, (1280, 800), R, T)
    cal.setStereoLeft(S.CAM_B, R1.tolist())
    cal.setStereoRight(S.CAM_C, Rr.tolist())
if a.dry_run:
    cal.eepromToJsonFile('/tmp/oak_flash_dryrun.json'); print('dry run -> /tmp/oak_flash_dryrun.json'); raise SystemExit
with dai.Device(dai.DeviceInfo(a.device)) as dev:
    dev.flashCalibration(cal)
    c = dev.readCalibration()
    print('flashed; baseline cm', round(c.getBaselineDistance(S.CAM_C, S.CAM_B), 3),
          'left fx', round(c.getCameraIntrinsics(S.CAM_B, 1280, 800)[0][0], 1))
