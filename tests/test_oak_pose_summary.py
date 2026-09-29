import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import oak_pose_summary as ops  # noqa: E402


def _det(z, hips_vis, ankles_vis):
    kp = [[0.0, 0.0, 0.9] for _ in range(17)]
    for i in (11, 12):
        kp[i][2] = hips_vis
    for i in (15, 16):
        kp[i][2] = ankles_vis
    return {'label': 'person', 'conf': 0.8, 'bbox': [0, 0, 1, 1], 'xyz_m': {'x': 0, 'y': 0, 'z': z},
            'keypoints': kp}


def test_summary_bins_visibility_and_rates(tmp_path):
    log = tmp_path / 'x.ndjson'
    rows = []
    for k in range(50):                         # 10 s at 5 Hz
        dets = [_det(1.2, 0.9, 0.1)] if k < 25 else ([] if k % 2 else [_det(2.5, 0.9, 0.9)])
        rows.append({'t_mono': 100 + k * 0.2, 't_wall': 0, 'model': 'yolo26n-pose', 'dets': dets})
    log.write_text(''.join(json.dumps(r) + '\n' for r in rows))
    s = ops.summarize(ops.load(log))
    assert s['model'] == 'yolo26n-pose' and abs(s['nn_hz'] - 5.0) < 0.01
    v = s['visibility']
    assert v['1-1.5m']['n'] == 25 and v['1-1.5m']['hips'] == 1.0 and v['1-1.5m']['ankles'] == 0.0
    assert v['2.2-3m']['ankles'] == 1.0
    seg = ops.summarize(ops.load(log, t_from=5.0))
    assert '1-1.5m' not in seg['visibility']


class _NN:
    def __init__(self, flat):
        self.flat = flat

    def getLayerFp16(self, name):
        return self.flat

    getFirstLayerFp16 = lambda self: self.flat  # noqa: E731


def test_driver_logs_every_packet_including_empty(tmp_path):
    import oakd_driver
    cam = oakd_driver.OakDCamera(spatial_config=str(ROOT / 'src' / 'blobs' / 'yolo26n-pose' / 'config.json'))
    cam.set_detection_log(tmp_path / 'd.ndjson', 'yolo26n-pose')
    cam._process_nn(_NN(np.zeros(56 * 6300, np.float16)))   # decodes, no detections
    cam._process_nn(_NN(np.zeros(10, np.float16)))          # short packet: stamped, empty
    cam.cleanup()
    lines = (tmp_path / 'd.ndjson').read_text().splitlines()
    assert len(lines) == 2 and all(json.loads(l)['dets'] == [] for l in lines)


def test_every_blob_config_loads_without_error(caplog):
    import oakd_driver
    for cfg in sorted((ROOT / 'src' / 'blobs').glob('*/config.json')):
        caplog.clear()
        cam = oakd_driver.OakDCamera(spatial_config=str(cfg))
        assert not [r for r in caplog.records if r.levelname == 'ERROR'], cfg
        assert cam._nn_rows in (85, 56, 116), cfg
