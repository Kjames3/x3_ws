"""Compare side-detector CPU pre/postprocessing using synthetic inputs.

Run with python3 src/tools/perf/trt_side_bench.py [--iterations 300].
TensorRT inference and CUDA synchronization are outside the timed stages.
"""

import argparse
from pathlib import Path
from statistics import median
import sys
from time import perf_counter

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from trt_detector import TRTDetector


def cpu_detector(fast):
    detector = TRTDetector.__new__(TRTDetector)
    detector.fast = fast
    detector.input_h = detector.input_w = 640
    detector.conf_thres = 0.4
    detector.iou_thres = 0.45
    detector._h_input = np.empty((1, 3, 640, 640), dtype=np.float32)
    if fast:
        detector._resized = np.empty((640, 640, 3), dtype=np.uint8)
    return detector


def timing_pair(legacy, fast, iterations):
    samples = {legacy: [], fast: []}
    for _ in range(10):
        legacy()
        fast()
    for repeat in range(5):
        functions = (legacy, fast) if repeat % 2 == 0 else (fast, legacy)
        for fn in functions:
            start = perf_counter()
            for _ in range(iterations):
                fn()
            samples[fn].append((perf_counter() - start) * 1000 / iterations)
    return median(samples[legacy]), median(samples[fast])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=300)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be positive")

    cv2.setNumThreads(1)
    rng = np.random.default_rng(26)
    frame = rng.integers(0, 256, (800, 1280, 3), dtype=np.uint8)
    end2end = rng.random((1, 300, 6), dtype=np.float32)
    end2end[..., :4] *= 640
    end2end[..., 5] = rng.integers(0, 80, 300)
    end2end[0, :5, 5] = 0
    classic = rng.random((1, 84, 8400), dtype=np.float32)
    classic[:, :4] *= 640
    classic[:, 4:] *= 0.3
    classic[0, 4, :5] = 0.8
    classic[0, 5:12, 5:10] = 0.7

    legacy, fast = cpu_detector(False), cpu_detector(True)

    def pre_legacy():
        return legacy._preprocess(frame)

    def pre_fast():
        return fast._preprocess(frame)

    def post_legacy(raw):
        boxes, scores, classes = legacy._postprocess(raw, 1280, 800)
        people = classes == 0
        return boxes[people], scores[people], classes[people]

    def post_fast(raw):
        return fast._postprocess(raw, 1280, 800, class_zero_only=True)

    def combined_legacy():
        pre_legacy()
        return post_legacy(end2end)

    def combined_fast():
        pre_fast()
        return post_fast(end2end)

    def report(label, legacy_fn, fast_fn):
        old_ms, new_ms = timing_pair(legacy_fn, fast_fn, args.iterations)
        print(f"{label:28s} {old_ms:10.4f} {new_ms:10.4f} {old_ms / new_ms:9.2f}x")

    orig_pixels, fast_pixels = pre_legacy(), pre_fast()
    np.testing.assert_allclose(orig_pixels, fast_pixels, atol=1e-7, rtol=0)
    print(f"OpenCV {cv2.__version__}, NumPy {np.__version__}, OpenCV threads {cv2.getNumThreads()}")
    print(f"Max preprocessing delta {np.max(np.abs(orig_pixels - fast_pixels)):.2g}")
    print(f"{'Stage':28s} {'Legacy ms':>10s} {'Fast ms':>10s} {'Speedup':>10s}")
    report("Preprocess 1280x800", pre_legacy, pre_fast)
    legacy._end2end = fast._end2end = True
    for expected, actual in zip(post_legacy(end2end), post_fast(end2end)):
        np.testing.assert_array_equal(expected, actual)
    report("YOLO26 postprocess", lambda: post_legacy(end2end), lambda: post_fast(end2end))
    report("YOLO26 pre + post", combined_legacy, combined_fast)
    legacy._end2end = fast._end2end = False
    for expected, actual in zip(post_legacy(classic), post_fast(classic)):
        np.testing.assert_array_equal(expected, actual)
    report("Classic YOLO postprocess", lambda: post_legacy(classic), lambda: post_fast(classic))


if __name__ == "__main__":
    main()
