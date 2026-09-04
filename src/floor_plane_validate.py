#!/usr/bin/env python3
"""Item 6: validate the level-camera assumption used by ground rejection.

Points the OAK at empty floor and reconstructs, for every valid depth pixel,
the height it implies under the deployed model:

    row_ray = (row - cy) / fy
    height  = CAMERA_HEIGHT_M - row_ray * Z          (pitch assumed 0)

For a level camera over a level floor the reconstructed floor height is 0 at
every range. A tilt shows up as a LINEAR RAMP of height against range, and the
residual pitch is exactly atan(slope) -- see the derivation in the report.

Cross-checks that geometric result against the OAK's own IMU gravity vector,
which measures tilt independently of any depth or intrinsics assumption.

Run with x3_server stopped (it holds the OAK device).
"""
import argparse, json, math, os, sys, time
import numpy as np

try:
    import depthai as dai
except ImportError:
    sys.exit("depthai not importable -- run this on the robot")

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(os.path.dirname(HERE), "config", "camera_ground_plane.json")
NN_W, NN_H = 480, 640           # CAM_A-aligned depth, matches oakd_driver spatial mode
Z_MIN, Z_MAX = 0.5, 4.0         # the estimator's valid depth band


def build_pipeline():
    p = dai.Pipeline()
    rgb = p.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setPreviewSize(NN_W, NN_H)
    rgb.setInterleaved(False)

    ml = p.create(dai.node.MonoCamera); ml.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mr = p.create(dai.node.MonoCamera); mr.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for m in (ml, mr):
        m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    st = p.create(dai.node.StereoDepth)
    st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    st.setLeftRightCheck(True)
    st.setSubpixel(True)
    st.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    st.setOutputSize(NN_W, NN_H)
    ml.out.link(st.left); mr.out.link(st.right)

    xd = p.create(dai.node.XLinkOut); xd.setStreamName("depth"); st.depth.link(xd.input)

    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
    imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
    xi = p.create(dai.node.XLinkOut); xi.setStreamName("imu"); imu.out.link(xi.input)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=60, help="depth frames to accumulate")
    ap.add_argument("--bin", type=float, default=0.25, help="range bin width (m)")
    ap.add_argument("--pct", type=float, default=16.0,
                    help="per-bin height percentile to treat as floor; the floor is the "
                         "LOWEST surface, so a low percentile rejects objects and walls "
                         "that a median would be dragged onto")
    ap.add_argument("--z-max", type=float, default=None,
                    help="ignore bins beyond this range (exclude a back wall)")
    ap.add_argument("--json-out", default="/tmp/floor_plane_report.json")
    args = ap.parse_args()

    cfg = {"camera_height_m": 0.210, "camera_pitch_deg": 0.0}
    try:
        with open(CFG) as f:
            cfg.update(json.load(f))
        print(f"config: {CFG}")
    except Exception as e:
        print(f"WARNING: could not read {CFG} ({e}); using built-in defaults")
    H = float(cfg["camera_height_m"])
    cfg_pitch = float(cfg.get("camera_pitch_deg", 0.0))
    print(f"assumed camera height {H:.3f} m, configured pitch {cfg_pitch:.3f} deg\n")

    with dai.Device(build_pipeline()) as dev:
        calib = dev.readCalibration()
        M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, NN_W, NN_H)
        fx, fy = float(M[0][0]), float(M[1][1])
        cx, cy = float(M[0][2]), float(M[1][2])
        print(f"CAM_A intrinsics @ {NN_W}x{NN_H}: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        qd = dev.getOutputQueue("depth", 4, False)
        qi = dev.getOutputQueue("imu", 8, False)

        accels = []
        sum_h = None; sum_h2 = None; cnt = None
        got = 0
        t0 = time.time()
        while got < args.frames and time.time() - t0 < 60:
            pkt = qd.tryGet()
            if pkt is not None:
                Z = pkt.getFrame().astype(np.float32) / 1000.0
                rows = np.arange(Z.shape[0], dtype=np.float32)
                # Same form as VelocityEstimator._height_band_mask: applying the
                # CONFIGURED pitch means what we fit below is the RESIDUAL after
                # correction, so a correct config drives the slope to zero.
                row_term = (((rows - cy) / fy) * math.cos(math.radians(cfg_pitch))
                            + math.sin(math.radians(cfg_pitch)))[:, None]
                height = H - row_term * Z
                valid = (Z >= Z_MIN) & (Z <= Z_MAX) & np.isfinite(Z)
                if sum_h is None:
                    sum_h = np.zeros_like(Z); sum_h2 = np.zeros_like(Z)
                    cnt = np.zeros(Z.shape, dtype=np.int32)
                    Zsum = np.zeros_like(Z)
                sum_h += np.where(valid, height, 0.0)
                sum_h2 += np.where(valid, height * height, 0.0)
                Zsum += np.where(valid, Z, 0.0)
                cnt += valid
                got += 1
            ip = qi.tryGet()
            if ip is not None:
                for p_ in ip.packets:
                    a = p_.acceleroMeter
                    accels.append((float(a.x), float(a.y), float(a.z)))
            time.sleep(0.005)

    if got == 0 or cnt is None or cnt.sum() == 0:
        sys.exit("no valid depth pixels captured -- is the floor in view and lit?")
    print(f"accumulated {got} frames, {int(cnt.sum())} valid pixel-samples\n")

    ok = cnt > 0
    mean_h = np.where(ok, sum_h / np.maximum(cnt, 1), np.nan)
    mean_Z = np.where(ok, Zsum / np.maximum(cnt, 1), np.nan)
    hs = mean_h[ok].ravel(); zs = mean_Z[ok].ravel()

    # ---- height vs range, binned; median per bin resists the odd wall/object
    edges = np.arange(Z_MIN, Z_MAX + args.bin, args.bin)
    idx = np.digitize(zs, edges) - 1
    zcap = args.z_max if args.z_max else Z_MAX
    print(f"{'range (m)':>14} {'n':>8} {'floor h (m)':>13} {'median':>9} {'spread':>8}  flag")
    print("-" * 70)
    bz, bh = [], []
    for b in range(len(edges) - 1):
        sel = idx == b
        n = int(sel.sum())
        if n < 200:
            continue
        hb = hs[sel]
        floor_h = float(np.percentile(hb, args.pct))
        med = float(np.median(hb))
        spread = float(np.percentile(hb, 84) - np.percentile(hb, 16))
        lo, hi = edges[b], edges[b + 1]
        # A wide p16..p84 spread means the bin holds more than the floor.
        bad = spread > 0.10 or hi > zcap
        flag = ("WALL/OBJ" if spread > 0.10 else "") + (" >z-max" if hi > zcap else "")
        print(f"  {lo:4.2f} - {hi:4.2f} {n:8d} {floor_h:13.4f} {med:9.3f} {spread:8.3f}  {flag}")
        if not bad:
            bz.append(0.5 * (lo + hi)); bh.append(floor_h)

    if len(bz) < 3:
        sys.exit("\ntoo few populated range bins to fit a slope -- capture more floor")

    bz = np.array(bz); bh = np.array(bh)
    print(f"\nfitting {len(bz)} clean bins (spread<=0.10 m, range<={zcap:.2f} m)"
          f" using the p{args.pct:g} floor height")
    # Theil-Sen: median of pairwise slopes, immune to a few bad bins.
    pairs = [(bh[j] - bh[i]) / (bz[j] - bz[i])
             for i in range(len(bz)) for j in range(i + 1, len(bz))]
    ts_slope = float(np.median(pairs))
    ts_int = float(np.median(bh - ts_slope * bz))
    slope, intercept = np.polyfit(bz, bh, 1)
    print(f"  least-squares slope {slope:+.5f} | Theil-Sen slope {ts_slope:+.5f} m/m")
    slope, intercept = ts_slope, ts_int
    resid = bh - (slope * bz + intercept)
    rms = float(np.sqrt(np.mean(resid ** 2)))
    pitch_deg = math.degrees(math.atan(slope))

    print("\n" + "=" * 56)
    print("GEOMETRIC FIT   height = slope*range + intercept")
    print(f"  slope        {slope:+.5f} m/m")
    print(f"  intercept    {intercept:+.5f} m")
    print(f"  fit RMS      {rms:.5f} m   (high => floor not planar / objects in view)")
    total_pitch = cfg_pitch + pitch_deg
    print(f"  => residual pitch  {pitch_deg:+.3f} deg  (on top of the configured "
          f"{cfg_pitch:+.3f} deg)")
    print(f"  => TOTAL pitch     {total_pitch:+.3f} deg   (positive = camera nose DOWN)")

    imu_pitch = imu_roll = None
    if accels:
        a = np.median(np.array(accels), axis=0)
        n = float(np.linalg.norm(a))
        print("\nIMU GRAVITY CROSS-CHECK (independent of depth + intrinsics)")
        print(f"  median accel  x={a[0]:+.3f} y={a[1]:+.3f} z={a[2]:+.3f} m/s^2  |a|={n:.3f}")
        if abs(n - 9.81) > 1.5:
            print("  !! |a| is not ~9.81 -- robot moving or IMU unreliable; ignore this check")
        else:
            # OAK optical convention: X right, Y down, Z forward.
            # Level camera => gravity lies along +Y only.
            imu_pitch = math.degrees(math.atan2(a[2], a[1]))
            imu_roll = math.degrees(math.atan2(a[0], a[1]))
            print(f"  pitch {imu_pitch:+.3f} deg (nose down +), roll {imu_roll:+.3f} deg")
            dom = int(np.argmax(np.abs(a)))
            if dom != 1:
                print(f"  !! gravity dominates axis {'XYZ'[dom]}, not Y -- the camera is NOT")
                print("     mounted in the orientation the height model assumes. Stop and")
                print("     resolve this before trusting any slope above.")

    print("\n" + "=" * 56)
    print("VERDICT")
    if abs(pitch_deg) < 0.5:
        print(f"  Residual {pitch_deg:+.3f} deg < 0.5 deg: configured pitch "
              f"{cfg_pitch:+.3f} deg is CORRECT, floor reconstructs flat.")
    else:
        err = math.tan(math.radians(pitch_deg)) * 4.0
        print(f"  Residual pitch {pitch_deg:+.3f} deg exceeds 0.5 deg.")
        print(f"  At 4 m that is {err:+.3f} m of height error against the 0.15 m floor")
        print(f"  threshold. Set camera_pitch_deg to {total_pitch:.3f} in")
        print(f"  {CFG} and re-run to confirm the residual drops.")
    if imu_pitch is not None:
        d = abs(imu_pitch - total_pitch)
        print(f"  geometry vs IMU disagree by {d:.3f} deg", end="")
        print("  (consistent)" if d < 1.0 else "  <-- INVESTIGATE, they should agree")

    with open(args.json_out, "w") as f:
        json.dump({"frames": got, "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                   "camera_height_m": H, "slope": float(slope),
                   "intercept": float(intercept), "fit_rms_m": rms,
                   "configured_pitch_deg": cfg_pitch,
                   "residual_pitch_deg": pitch_deg, "total_pitch_deg": total_pitch, "imu_pitch_deg": imu_pitch,
                   "imu_roll_deg": imu_roll,
                   "bins": [{"range_m": float(z), "median_height_m": float(h)}
                            for z, h in zip(bz, bh)]}, f, indent=2)
    print(f"\nreport written to {args.json_out}")


if __name__ == "__main__":
    main()
