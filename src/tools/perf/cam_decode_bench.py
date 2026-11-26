"""Time read() vs grab()+retrieve() on a synthetic 1280x800 MJPEG source.

The Arducam capture thread calls cap.read() for every sensor frame; with
X3_CAM_DECODE_ON_DEMAND=1 it grab()s every frame and retrieve()s only at
X3_CAM_DECODE_HZ (cam_decode.py). This bench estimates the saving off-robot.

Backends differ in WHERE the decode happens, so three modes are reported:

* ffmpeg  - cv2.VideoCapture on an MJPEG .avi. FFmpeg decodes inside grab();
            retrieve() only converts colour, so this UNDER-states the saving.
* v4l2    - the same file in raw mode (CAP_PROP_FORMAT=-1): grab() returns the
            compressed packet and the decode is cv2.imdecode in "retrieve".
            That is how the V4L2 backend splits an MJPEG camera
            (grab = VIDIOC_DQBUF, retrieve = JPEG decode), so it is the model
            for the robot.
* rotate  - the left camera's 640x400 cv2.rotate(ROTATE_180), skipped with the
            decode.

Run on the laptop, numbers are not the Orin's; the ratio is what carries over.

    python3 src/tools/perf/cam_decode_bench.py [--frames 300] [--fps 30] [--hz 10]
"""

import argparse
import os
import tempfile
import time

import cv2
import numpy as np


def make_mjpeg(path, n, w=1280, h=800, fps=30.0, quality=85):
    """A textured MJPEG .avi (gradients + moving shapes + mild noise, so JPEG
    frames are a realistic ~100-200 kB, not trivially compressible)."""
    vw = cv2.VideoWriter(path, cv2.VideoWriter.fourcc(*'MJPG'), fps, (w, h))
    vw.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)
    rng = np.random.default_rng(0)
    yy, xx = np.mgrid[0:h, 0:w]
    base = np.dstack([(xx * 255 // w), (yy * 255 // h), ((xx + yy) * 255 // (w + h))]).astype(np.uint8)
    noise = rng.integers(0, 24, size=(h, w, 3), dtype=np.uint8)
    for i in range(n):
        img = cv2.add(base, noise)
        cv2.circle(img, ((40 * i) % w, h // 2), 120, (255, 255, 255), -1)
        cv2.rectangle(img, (w // 3, (17 * i) % h), (w // 3 + 200, (17 * i) % h + 150), (0, 0, 0), 4)
        cv2.putText(img, f'frame {i}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 5)
        vw.write(img)
    vw.release()


def _timed(fn):
    c0, w0 = time.process_time(), time.perf_counter()
    n_dec = fn()
    return time.process_time() - c0, time.perf_counter() - w0, n_dec


def bench_ffmpeg(path, every):
    def legacy():
        cap = cv2.VideoCapture(path, cv2.CAP_FFMPEG)
        n = 0
        while True:
            ok, _ = cap.read()
            if not ok:
                break
            n += 1
        cap.release()
        return n

    def on_demand():
        cap = cv2.VideoCapture(path, cv2.CAP_FFMPEG)
        k = n = 0
        while cap.grab():
            if k % every == 0:
                ok, _ = cap.retrieve()
                n += ok
            k += 1
        cap.release()
        return n
    return _timed(legacy), _timed(on_demand)


def _raw_cap(path):
    cap = cv2.VideoCapture(path, cv2.CAP_FFMPEG)
    if not cap.set(cv2.CAP_PROP_FORMAT, -1):
        return None
    return cap


def bench_v4l2_model(path, every):
    if _raw_cap(path) is None:
        return None

    def legacy():
        cap = _raw_cap(path)
        n = 0
        while True:
            ok, pkt = cap.read()          # grab + raw packet
            if not ok:
                break
            if cv2.imdecode(pkt, cv2.IMREAD_COLOR) is not None:
                n += 1
        cap.release()
        return n

    def on_demand():
        cap = _raw_cap(path)
        k = n = 0
        while cap.grab():
            if k % every == 0:
                ok, pkt = cap.retrieve()
                if ok and cv2.imdecode(pkt, cv2.IMREAD_COLOR) is not None:
                    n += 1
            k += 1
        cap.release()
        return n
    return _timed(legacy), _timed(on_demand)


def bench_rotate(n, every):
    img = np.random.default_rng(1).integers(0, 255, size=(400, 640, 3), dtype=np.uint8)

    def legacy():
        for _ in range(n):
            cv2.rotate(img, cv2.ROTATE_180)
        return n

    def on_demand():
        for k in range(0, n, every):
            cv2.rotate(img, cv2.ROTATE_180)
        return len(range(0, n, every))
    return _timed(legacy), _timed(on_demand)


def report(name, res, n, fps):
    (cl, wl, nl), (co, wo, no) = res
    seconds = n / fps                     # stream duration this many frames represents
    print(f"{name:7s} every frame (read): {nl:4d} decodes, {1e3 * cl / max(n, 1):6.2f} ms CPU/frame, "
          f"{100 * cl / seconds:5.1f} % of one core at {fps:.0f} fps")
    print(f"{'':7s} gated (grab+retr.): {no:4d} decodes, {1e3 * co / max(n, 1):6.2f} ms CPU/frame, "
          f"{100 * co / seconds:5.1f} % of one core   -> {cl / max(co, 1e-9):4.1f}x less CPU")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', type=int, default=300)
    ap.add_argument('--fps', type=float, default=30.0, help='sensor frame rate being modelled')
    ap.add_argument('--hz', type=float, default=10.0, help='X3_CAM_DECODE_HZ being modelled')
    ap.add_argument('--threads', type=int, default=1,
                    help='cv2.setNumThreads (1 = like one capture thread; 0 = OpenCV default)')
    a = ap.parse_args()
    cv2.setNumThreads(a.threads)
    every = max(1, round(a.fps / a.hz))
    print(f"OpenCV {cv2.__version__}, {a.frames} frames 1280x800 MJPEG, decode every {every} "
          f"({a.fps:.0f} fps -> {a.fps / every:.1f} Hz), cv2 threads {a.threads}. LAPTOP numbers.")
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'synthetic_1280x800.avi')
        make_mjpeg(path, a.frames, fps=a.fps)
        print(f"synthetic file {os.path.getsize(path) / a.frames / 1e3:.0f} kB/frame")
        report('ffmpeg', bench_ffmpeg(path, every), a.frames, a.fps)
        print("        (FFmpeg decodes inside grab(): this mode understates the V4L2 saving)")
        v = bench_v4l2_model(path, every)
        if v is None:
            print("v4l2    raw packet mode (CAP_PROP_FORMAT=-1) unsupported by this OpenCV build")
        else:
            report('v4l2', v, a.frames, a.fps)
    report('rotate', bench_rotate(a.frames, every), a.frames, a.fps)


if __name__ == '__main__':
    main()
