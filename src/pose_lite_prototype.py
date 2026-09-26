#!/usr/bin/env python3
"""Laptop-only Pose Lite evaluation. No ROS, robot controls or GUI integration."""
import argparse
from datetime import datetime
import json
import math
import threading
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / '.cache/pose-lite/pose_landmarker_lite.task'
NAMES = ('nose left_eye_inner left_eye left_eye_outer right_eye_inner right_eye '
         'right_eye_outer left_ear right_ear mouth_left mouth_right left_shoulder '
         'right_shoulder left_elbow right_elbow left_wrist right_wrist left_pinky '
         'right_pinky left_index right_index left_thumb right_thumb left_hip '
         'right_hip left_knee right_knee left_ankle right_ankle left_heel '
         'right_heel left_foot_index right_foot_index').split()


def landmark_dict(point):
    return {key: float(getattr(point, key)) if getattr(point, key, None) is not None else None
            for key in ('x', 'y', 'z', 'visibility', 'presence')}


def encode_result(result, timestamp_ms, elapsed_ms, width, height):
    return {'timestamp_ms': timestamp_ms, 'inference_ms': elapsed_ms,
            'image_width': width, 'image_height': height,
            'poses': [{'pose_index': i,  # ordering is NOT a persistent person ID
                       'image_landmarks': [landmark_dict(p) for p in image],
                       'hip_relative_world_landmarks_m': [landmark_dict(p) for p in world]}
                      for i, (image, world) in enumerate(zip(result.pose_landmarks, result.pose_world_landmarks))]}


def overlay(frame, result, elapsed_ms, visibility):
    frame = frame.copy()
    height, width = frame.shape[:2]
    for pose in result.pose_landmarks:
        def visible(p):
            return (p.visibility or 0) >= visibility and (p.presence or 0) >= visibility and 0 <= p.x <= 1 and 0 <= p.y <= 1
        def pixel(p):
            return round(p.x * (width - 1)), round(p.y * (height - 1))
        for edge in mp.tasks.vision.PoseLandmarksConnections.POSE_LANDMARKS:
            a, b = pose[edge.start], pose[edge.end]
            if visible(a) and visible(b):
                cv2.line(frame, pixel(a), pixel(b), (0, 220, 255), 2, cv2.LINE_AA)
        for p in pose:
            if visible(p):
                cv2.circle(frame, pixel(p), 3, (70, 255, 70), -1, cv2.LINE_AA)
    cv2.putText(frame, f'Pose Lite | {len(result.pose_landmarks)} person(s) | inference {elapsed_ms:.1f} ms',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
    return frame


class LatestCapture:
    """Drain live video in a reader thread; keep at most one unprocessed frame."""
    def __init__(self, capture):
        self.capture = capture
        self.condition = threading.Condition()
        self.latest = None
        self.finished = False
        self.stop = threading.Event()
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        try:
            while not self.stop.is_set():
                ok, frame = self.capture.read()
                if not ok:
                    break
                with self.condition:
                    self.latest = (frame, round((time.monotonic() - self.started) * 1000))
                    self.condition.notify()
        finally:
            self.capture.release()
            with self.condition:
                self.finished = True
                self.condition.notify_all()

    def get(self):
        with self.condition:
            if not self.condition.wait_for(lambda: self.latest is not None or self.finished, timeout=5):
                raise RuntimeError('No camera frame received for 5 seconds')
            value, self.latest = self.latest, None
            return value

    def close(self):
        self.stop.set()
        self.thread.join(timeout=4)


def frames(source, rate, max_frames):
    path = Path(source)
    if path.is_file() and path.suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.webp'}:
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f'Cannot decode image: {path}')
        yield image, 0
        return
    live = source.isdecimal() or '://' in source
    if '://' in source:
        capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG,
                                   [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000])
    else:
        capture = cv2.VideoCapture(int(source) if source.isdecimal() else source)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f'Cannot open source: {source}')
    reader = LatestCapture(capture) if live else None
    fps = capture.get(cv2.CAP_PROP_FPS) if not live else 0
    if not live and (not math.isfinite(fps) or fps <= 0):
        capture.release()
        raise RuntimeError('Video frame rate is unavailable; transcode it to a constant frame rate first')
    last_timestamp, next_sample, count, index = -1, 0, 0, 0
    try:
        while not max_frames or count < max_frames:
            if live:
                if count:
                    time.sleep(max(0, next_sample - time.monotonic()))
                item = reader.get()
                if item is None:
                    raise RuntimeError('Live video stream ended')
                frame, timestamp = item
                next_sample = time.monotonic() + 1 / rate
            else:
                ok, frame = capture.read()
                if not ok:
                    break
                timestamp = round(index * 1000 / fps)
                index += 1
                if timestamp + 0.001 < next_sample:
                    continue
                next_sample += 1000 / rate
            timestamp = max(last_timestamp + 1, timestamp)
            last_timestamp = timestamp
            count += 1
            yield frame, timestamp
    finally:
        if reader:
            reader.close()
        else:
            capture.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', help='Image/video path, camera index (0), or RTSP/HTTP video URL')
    parser.add_argument('--model', type=Path, default=MODEL)
    parser.add_argument('--fps', type=float, default=10, help='Maximum live inference Hz / offline sampling Hz')
    parser.add_argument('--num-poses', type=int, default=1)
    parser.add_argument('--visibility', type=float, default=0.5, help='Minimum confidence for drawing each joint')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path, help='New output directory (never overwrites an existing run)')
    parser.add_argument('--self-test', action='store_true', help='Infer on blank frames without accessing any camera')
    args = parser.parse_args()
    if not args.self_test and not args.source:
        parser.error('--source is required unless --self-test is used')
    if not math.isfinite(args.fps) or not 0 < args.fps <= 120 or args.num_poses < 1 or args.max_frames < 0 or not 0 <= args.visibility <= 1:
        parser.error('Use fps in (0,120], num-poses >= 1, max-frames >= 0, visibility in [0,1]')
    if not args.model.is_file():
        parser.error('Model missing. Run bash scripts/setup_pose_lite.sh')
    options = mp.tasks.vision.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(args.model), delegate=mp.tasks.BaseOptions.Delegate.CPU),
        running_mode=mp.tasks.vision.RunningMode.VIDEO, num_poses=args.num_poses,
        output_segmentation_masks=False)
    run = args.output or ROOT / '.cache/pose-lite/runs' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    run.mkdir(parents=True, exist_ok=False)
    metadata = {'model': 'MediaPipe Pose Landmarker Lite float16 v1', 'mediapipe': mp.__version__,
                'source': 'synthetic blank frames' if args.self_test else args.source,
                'requested_fps': args.fps, 'num_poses': args.num_poses,
                'landmark_names': NAMES, 'world_frame': 'hip-centred model estimate; NOT robot/map coordinates',
                'timestamp_basis': 'decoded-frame receipt time for live; frame index/FPS for recorded video',
                'identity': 'pose_index is per-frame ordering, not a tracking ID'}
    (run / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    source = iter([(np.zeros((480, 640, 3), np.uint8), i * 100) for i in range(3)]) if args.self_test else frames(args.source, args.fps, args.max_frames)
    timings, detections = [], 0
    annotated = None
    started = time.monotonic()
    try:
        with mp.tasks.vision.PoseLandmarker.create_from_options(options) as detector, (run / 'poses.ndjson').open('w') as log:
            for frame, timestamp in source:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                before = time.perf_counter()
                result = detector.detect_for_video(image, timestamp)
                elapsed = (time.perf_counter() - before) * 1000
                timings.append(elapsed)
                detections += bool(result.pose_landmarks)
                row = encode_result(result, timestamp, elapsed, frame.shape[1], frame.shape[0])
                log.write(json.dumps(row, allow_nan=False) + '\n')
                log.flush()
                annotated = overlay(frame, result, elapsed, args.visibility)
                if not args.headless and not args.self_test:
                    cv2.imshow('Laptop Pose Lite - Q or Esc to stop', annotated)
                    still = Path(args.source).suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.webp'}
                    if cv2.waitKey(0 if still else 1) & 0xff in (27, ord('q')):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(source, 'close'):
            source.close()
        if not args.headless and not args.self_test:
            cv2.destroyAllWindows()
        if annotated is not None:
            cv2.imwrite(str(run / 'last-frame.jpg'), annotated)
        summary = {'frames_processed': len(timings), 'frames_with_pose': detections,
                   'elapsed_s': time.monotonic() - started,
                   'inference_ms_median': float(np.median(timings)) if timings else None,
                   'inference_ms_p95': float(np.percentile(timings, 95)) if timings else None}
        (run / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps({'output': str(run), **summary}, indent=2))
    if not timings:
        raise RuntimeError('No frames processed')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
