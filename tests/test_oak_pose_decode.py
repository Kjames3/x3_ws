"""The driver's pose decode on real ONNX output (skips without onnxruntime).

Run with the export venv: .cache/onnx-export/bin/python -m pytest tests/test_oak_pose_decode.py
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
ort = pytest.importorskip('onnxruntime')
cv2 = pytest.importorskip('cv2')
SAMPLE = ROOT / '.cache' / 'pose-lite' / 'sample-person.jpg'


class _NN:
    def __init__(self, flat):
        self.flat = flat

    def getLayerFp16(self, name):
        return self.flat

    def getFirstLayerFp16(self):
        return self.flat


@pytest.mark.parametrize('name', ['yolo11n-pose', 'yolo26n-pose'])
def test_pose_blob_output_decodes_to_person_with_keypoints(name):
    if not SAMPLE.exists():
        pytest.skip('sample image not cached')
    import oakd_driver
    img = cv2.resize(cv2.imread(str(SAMPLE)), (480, 640))
    x = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    sess = ort.InferenceSession(str(ROOT / 'models' / 'onnx' / f'{name}-640x480.onnx'))
    flat = sess.run(None, {'images': x})[0].astype(np.float16).ravel()   # device emits FP16

    cam = oakd_driver.OakDCamera(spatial_config=str(ROOT / 'src' / 'blobs' / name / 'config.json'))
    assert cam.nn_kpts == 17 and cam._nn_rows == 56
    cam._process_nn(_NN(flat))
    dets = cam.get_spatial_detections()
    assert cam._decode_mode == ('cm', False)
    assert len(dets) == 1 and dets[0]['label'] == 'person'
    kp = np.array(dets[0]['keypoints'])
    assert kp.shape == (17, 3)
    x1, y1, x2, y2 = dets[0]['bbox']
    vis = kp[kp[:, 2] > 0.5]
    assert len(vis) >= 10
    assert (vis[:, 0] >= x1 - 20).all() and (vis[:, 0] <= x2 + 20).all()
    assert (vis[:, 1] >= y1 - 20).all() and (vis[:, 1] <= y2 + 20).all()
