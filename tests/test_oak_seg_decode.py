"""The driver's seg decode on real ONNX output (skips without onnxruntime).

Run with the export venv: .cache/onnx-export/bin/python -m pytest tests/test_oak_seg_decode.py
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
    def __init__(self, layers):
        self.layers = layers

    def getLayerFp16(self, name):
        return self.layers[name]

    def getFirstLayerFp16(self):
        return self.layers['output0']


@pytest.mark.parametrize('name', ['yolo26n-seg', 'yolo11n-seg'])
def test_seg_output_decodes_to_a_person_mask_and_mask_depth(name):
    if not SAMPLE.exists():
        pytest.skip('sample image not cached')
    import oakd_driver
    img = cv2.resize(cv2.imread(str(SAMPLE)), (480, 640))
    x = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    sess = ort.InferenceSession(str(ROOT / 'models' / 'onnx' / f'{name}-640x480.onnx'))
    o0, o1 = sess.run(None, {'images': x})
    cam = oakd_driver.OakDCamera(spatial_config=str(ROOT / 'src' / 'blobs' / name / 'config.json'))
    assert cam.nn_masks == 32 and cam._nn_rows == 116 and cam.nn_proto_shape == (32, 160, 120)

    # Reference silhouette from Ultralytics on the same input.
    from ultralytics import YOLO
    ref = YOLO(str(ROOT / 'models' / f'{name}.pt')).predict(img, imgsz=(640, 480), verbose=False,
                                                            device='cpu', classes=[0], retina_masks=True)[0]
    ref_mask = ref.masks.data[0].numpy() > 0.5

    # Depth: person (reference silhouette) at 2.0 m, everything else a wall at 4.0 m.
    depth = np.where(ref_mask, 2.0, 4.0).astype(np.float32)
    cam._latest_raw_depth = depth
    cam._fx = cam._fy = 677.0
    cam._cx, cam._cy = 240.0, 320.0
    cam._process_nn(_NN({'output0': o0.astype(np.float16).ravel(), 'output1': o1.astype(np.float16).ravel()}))
    dets = cam.get_spatial_detections()
    masks = cam.get_detection_masks()
    people = [i for i, d in enumerate(dets) if d['label'] == 'person']
    assert people, dets
    i = people[0]
    m = masks[i]
    assert m is not None and m.shape == (640, 480)
    iou = (m & ref_mask).sum() / (m | ref_mask).sum()
    assert iou > 0.7, iou
    assert abs(dets[i]['xyz_m']['z'] - 2.0) < 0.05, dets[i]['xyz_m']
    # The box-only estimate on the same depth, for comparison: pulled toward the wall.
    box_only = cam._locate(depth, *dets[i]['bbox'])
    print(f'{name}: mask IoU {iou:.3f}; mask depth {dets[i]["xyz_m"]["z"]:.3f} m, '
          f'box-only depth {box_only[2]:.3f} m')
