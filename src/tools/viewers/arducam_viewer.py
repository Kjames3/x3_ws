#!/usr/bin/env python3
"""Popup viewer for the Arducam UC-844 (OV9281 global-shutter mono USB camera).

Bench tool for judging it as a side camera. Shows the live image with a metrics
overlay (FPS, frame-interval jitter, exposure, brightness, saturation,
sharpness), plus three test aids:

* AprilTag (tag36h11) detection. Once the focal length is known it shows the
  tag's range, bearing and height relative to the camera, and from the known
  tag-centre height it infers the camera height above the floor.
* Focal-length / FOV calibration from the tag: hold the camera square-on at a
  tape-measured distance (--cal-dist, metres, lens to tag face) and press c.
  fx is averaged over 20 frames, HFOV/VFOV are printed and the result is saved
  to config/arducam_uc844_intrinsics.json (rescaled for other resolutions).
* YOLO person detection (y). With fx and a camera height known, each person
  gets a flat-floor range from where their box meets the floor, the way a
  side camera would range them. Assumes the camera is level.

    python3 src/tools/viewers/arducam_viewer.py
    python3 src/tools/viewers/arducam_viewer.py --cal-dist 1.00

Keys: q/Esc quit, c calibrate fx, y person detection, t tag detection,
      r cycle resolution, m MJPG/YUYV, h histogram, s snapshot,
      +/- manual exposure, a auto exposure.
"""
import argparse
import collections
import glob
import json
import math
import os
import time

import cv2
import numpy as np

RESOLUTIONS = [(1280, 800), (1280, 720), (640, 400), (320, 240)]
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
INTR_PATH = os.path.join(REPO, "config", "arducam_uc844_intrinsics.json")
YOLO_PATH = os.path.join(REPO, "src", "yolo_models", "default", "yolo26n.pt")
CAL_FRAMES = 20


def find_arducam():
    """Return the first /dev/videoN whose V4L2 name says Arducam (capture node)."""
    for p in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            name = open(os.path.join(p, "name")).read().strip()
            idx = int(open(os.path.join(p, "index")).read().strip())
        except OSError:
            continue
        if "arducam" in name.lower() and idx == 0:
            return int(p.rsplit("video", 1)[1]), name
    return None, None


def open_cam(dev, w, h, fps, fourcc):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def fourcc_str(cap):
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((v >> 8 * i) & 0xFF) for i in range(4))


def draw_text(img, lines, x=10, y=24, color=(0, 255, 0)):
    for i, t in enumerate(lines):
        yy = y + i * 22
        cv2.putText(img, t, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, t, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def draw_hist(img, gray):
    hist = cv2.calcHist([gray], [0], None, [64], [0, 256]).ravel()
    hist /= hist.max() + 1e-9
    hw, hh = 256, 80
    x0, y0 = img.shape[1] - hw - 10, img.shape[0] - hh - 10
    cv2.rectangle(img, (x0, y0), (x0 + hw, y0 + hh), (0, 0, 0), -1)
    for i, v in enumerate(hist):
        cv2.rectangle(img, (x0 + i * 4, y0 + hh - int(v * hh)),
                      (x0 + i * 4 + 3, y0 + hh), (0, 200, 255), -1)


def load_fx(w):
    """fx in pixels for image width w, from the saved calibration (scaled)."""
    try:
        d = json.load(open(INTR_PATH))
        return d["fx"] * w / d["width"]
    except (OSError, KeyError, ValueError):
        return None


def save_fx(fx, w, h, dist, tag):
    os.makedirs(os.path.dirname(INTR_PATH), exist_ok=True)
    d = {"fx": fx, "width": w, "height": h,
         "hfov_deg": math.degrees(2 * math.atan(w / 2 / fx)),
         "vfov_deg": math.degrees(2 * math.atan(h / 2 / fx)),
         "method": f"AprilTag {tag} m at {dist} m, square-on, pinhole, square pixels",
         "date": time.strftime("%Y-%m-%d")}
    json.dump(d, open(INTR_PATH, "w"), indent=2)
    return d


def tag_side_px(c):
    """Mean edge length in pixels of a 4x2 corner array."""
    return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


def tag_pose(c, fx, w, h, size):
    """solvePnP for one tag. Returns (x right, y down, z forward) in metres."""
    s = size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], np.float64)
    ok, _, t = cv2.solvePnP(obj, c.astype(np.float32), K, None,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return t.ravel() if ok else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", type=int, help="/dev/videoN (default: auto-detect Arducam)")
    ap.add_argument("--res", default="1280x800")
    ap.add_argument("--fps", type=int, default=120)
    ap.add_argument("--fourcc", default="MJPG", choices=["MJPG", "YUYV"])
    ap.add_argument("--tag-size", type=float, default=0.145,
                    help="black-square edge of the tag, metres")
    ap.add_argument("--tag-height", type=float, default=0.28,
                    help="tag centre height above the floor, metres")
    ap.add_argument("--cal-dist", type=float, default=1.0,
                    help="lens-to-tag distance used when you press c, metres")
    ap.add_argument("--fx", type=float, help="override focal length (px at --res)")
    ap.add_argument("--cam-height", type=float,
                    help="camera height for person ranging (default: from the tag)")
    ap.add_argument("--snap-dir", default=os.path.expanduser("~/arducam_snaps"))
    args = ap.parse_args()

    dev, name = (args.dev, None) if args.dev is not None else find_arducam()
    if dev is None:
        raise SystemExit("No Arducam found in /sys/class/video4linux; pass --dev N")
    print(f"Using /dev/video{dev} {name or ''}")

    w, h = map(int, args.res.lower().split("x"))
    res_i = RESOLUTIONS.index((w, h)) if (w, h) in RESOLUTIONS else 0
    fourcc = args.fourcc
    cap = open_cam(dev, w, h, args.fps, fourcc)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{dev}")

    tag_det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters())
    yolo = None
    show_tag, show_yolo, show_hist = True, False, True
    cal_samples = None
    cam_h_est = collections.deque(maxlen=30)

    win = "Arducam UC-844 (OV9281)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    stamps = collections.deque(maxlen=120)
    frames = fails = 0
    t_start = time.monotonic()
    proc_ms = 0.0
    msg, msg_until = "", 0.0

    while True:
        ok, frame = cap.read()
        now = time.monotonic()
        if not ok or frame is None:
            fails += 1
            if fails % 30 == 0:
                print(f"read failed x{fails}")
            if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                break
            continue
        frames += 1
        stamps.append(now)
        t0 = time.perf_counter()

        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fh, fw = gray.shape
        fx = args.fx * fw / w if args.fx else load_fx(fw)
        mean = float(gray.mean())
        sat = 100.0 * float(np.count_nonzero(gray >= 250)) / gray.size
        dark = 100.0 * float(np.count_nonzero(gray <= 5)) / gray.size
        sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if len(stamps) > 2:
            dt = np.diff(np.array(stamps)) * 1000.0
            fps, jit, dt_max = 1000.0 / dt.mean(), dt.std(), dt.max()
        else:
            fps = jit = dt_max = 0.0

        disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        info = []

        # ---- AprilTag ----
        if show_tag or cal_samples is not None:
            corners, ids, _ = tag_det.detectMarkers(gray)
            if ids is not None:
                for c, i in zip(corners, ids.ravel()):
                    c = c.reshape(4, 2)
                    cv2.polylines(disp, [c.astype(int)], True, (0, 255, 255), 2)
                    side = tag_side_px(c)
                    line = f"tag {i}: side {side:.0f} px"
                    if fx:
                        t = tag_pose(c, fx, fw, fh, args.tag_size)
                        if t is not None:
                            rng = float(np.linalg.norm(t))
                            brg = math.degrees(math.atan2(t[0], t[2]))
                            # camera level: tag centre is -y above the camera
                            cam_h = args.tag_height + float(t[1])
                            cam_h_est.append(cam_h)
                            line += (f"  range {rng:.3f} m  fwd {t[2]:.3f}  "
                                     f"brg {brg:+.1f} deg  cam h {cam_h:.3f} m")
                    info.append(line)
                    if cal_samples is not None:
                        cal_samples.append(side)
            if cal_samples is not None:
                info.append(f"CALIBRATING {len(cal_samples)}/{CAL_FRAMES} "
                            f"- hold square-on at {args.cal_dist} m")
                if len(cal_samples) >= CAL_FRAMES:
                    side = float(np.median(cal_samples))
                    fx_new = side * args.cal_dist / args.tag_size
                    d = save_fx(fx_new, fw, fh, args.cal_dist, args.tag_size)
                    args.fx = None
                    msg = (f"fx {fx_new:.1f} px  HFOV {d['hfov_deg']:.1f}  "
                           f"VFOV {d['vfov_deg']:.1f} deg  (saved)")
                    print(msg, "->", INTR_PATH)
                    msg_until = now + 8
                    cal_samples = None

        # ---- persons ----
        if show_yolo:
            if yolo is None:
                from ultralytics import YOLO
                yolo = YOLO(YOLO_PATH)
            cam_h = args.cam_height or (float(np.median(cam_h_est)) if cam_h_est else None)
            res = yolo.predict(disp, classes=[0], conf=0.35, verbose=False)[0]
            for b, cf in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy()):
                x1, y1, x2, y2 = b.astype(int)
                cv2.rectangle(disp, (x1, y1), (x2, y2), (255, 120, 0), 2)
                label = f"person {cf:.2f}"
                if fx and cam_h and y2 < fh - 3 and y2 - fh / 2 > 2:
                    z = fx * cam_h / (y2 - fh / 2)  # flat floor, level camera
                    label += f"  {z:.2f} m"
                elif y2 >= fh - 3:
                    label += "  feet cut off"
                cv2.putText(disp, label, (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 0), 2)

        aw, ah = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        fov = (f"fx {fx:.0f} px  HFOV {math.degrees(2 * math.atan(fw / 2 / fx)):.1f}  "
               f"VFOV {math.degrees(2 * math.atan(fh / 2 / fx)):.1f} deg" if fx
               else "fx unknown - press c at --cal-dist from the tag")
        draw_text(disp, [
            f"/dev/video{dev}  {aw}x{ah} {fourcc_str(cap)}  req {args.fps} fps",
            f"FPS {fps:6.1f}   dt jitter {jit:5.2f} ms   max dt {dt_max:5.1f} ms",
            f"frames {frames}  read fails {fails}  overlay {proc_ms:4.1f} ms",
            f"exposure {exp:.0f} (auto={cap.get(cv2.CAP_PROP_AUTO_EXPOSURE):.0f})",
            f"brightness {mean:5.1f}  sat {sat:4.1f}%  dark {dark:4.1f}%  sharp {sharp:7.1f}",
            fov,
        ] + info)
        if now < msg_until:
            draw_text(disp, [msg], y=fh - 20, color=(0, 200, 255))
        if show_hist:
            draw_hist(disp, gray)
        proc_ms = (time.perf_counter() - t0) * 1000.0
        cv2.imshow(win, disp)

        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
        elif k in (ord("r"), ord("m")):
            if k == ord("r"):
                res_i = (res_i + 1) % len(RESOLUTIONS)
            else:
                fourcc = "YUYV" if fourcc == "MJPG" else "MJPG"
            cap.release()
            rw, rh = RESOLUTIONS[res_i]
            cap = open_cam(dev, rw, rh, args.fps, fourcc)
            stamps.clear()
        elif k == ord("c"):
            cal_samples = []
        elif k == ord("y"):
            show_yolo = not show_yolo
        elif k == ord("t"):
            show_tag = not show_tag
        elif k == ord("h"):
            show_hist = not show_hist
        elif k == ord("s"):
            os.makedirs(args.snap_dir, exist_ok=True)
            path = os.path.join(args.snap_dir, time.strftime("arducam_%Y%m%d_%H%M%S.png"))
            cv2.imwrite(path, frame)
            print("saved", path)
        elif k in (ord("+"), ord("=")):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # V4L2 manual
            cap.set(cv2.CAP_PROP_EXPOSURE, max(1, exp) * 1.25)
        elif k == ord("-"):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, max(1, exp / 1.25))
        elif k == ord("a"):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # V4L2 aperture-priority auto

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
