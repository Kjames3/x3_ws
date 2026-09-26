"""Run with the isolated Pose Lite interpreter; no camera/model download needed."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('pose_lite', Path(__file__).resolve().parents[1] / 'src/pose_lite_prototype.py')
pose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose)


class PoseLiteTests(unittest.TestCase):
    def test_video_sampling_uses_source_time(self):
        with tempfile.TemporaryDirectory() as folder:
            file = str(Path(folder) / 'test.avi')
            writer = cv2.VideoWriter(file, cv2.VideoWriter_fourcc(*'MJPG'), 30, (64, 48))
            self.assertTrue(writer.isOpened())
            for _ in range(30):
                writer.write(np.zeros((48, 64, 3), np.uint8))
            writer.release()
            samples = list(pose.frames(file, 10, 0))
            self.assertEqual([t for _, t in samples], list(range(0, 1000, 100)))
            self.assertEqual(len(list(pose.frames(file, 10, 3))), 3)

    def test_live_reader_drops_backlog_and_releases_capture(self):
        class Capture:
            count = 0
            released = False
            def read(self):
                self.count += 1
                return (True, self.count) if self.count <= 100 else (False, None)
            def release(self):
                self.released = True
        capture = Capture()
        reader = pose.LatestCapture(capture)
        reader.thread.join(timeout=2)
        self.assertEqual(reader.get()[0], 100)
        self.assertIsNone(reader.get())
        reader.close()
        self.assertTrue(capture.released)

    def test_export_keeps_hip_coordinates_separate(self):
        point = SimpleNamespace(x=0.1, y=0.2, z=-0.1, visibility=0.9, presence=None)
        result = SimpleNamespace(pose_landmarks=[[point]], pose_world_landmarks=[[point]])
        row = pose.encode_result(result, 321, 12.3, 640, 480)
        self.assertEqual(row['timestamp_ms'], 321)
        self.assertEqual(row['poses'][0]['pose_index'], 0)
        self.assertIn('hip_relative_world_landmarks_m', row['poses'][0])
        self.assertIsNone(row['poses'][0]['image_landmarks'][0]['presence'])
        self.assertEqual(pose.encode_result(SimpleNamespace(pose_landmarks=[], pose_world_landmarks=[]), 0, 1, 2, 2)['poses'], [])


if __name__ == '__main__':
    unittest.main()
