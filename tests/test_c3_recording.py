"""C3 recorder: pre-NMS person boxes from the OAK decode, and the flag-driven packet log."""
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import c3_live  # noqa: E402
import oakd_driver  # noqa: E402


def _cam(scores, boxes):
    """An OakDCamera with only what _decode_nn touches, decoding to the given
    normalised cxcywh boxes (all class 0 = person)."""
    cam = object.__new__(oakd_driver.OakDCamera)
    cam._lock = threading.Lock()
    cam.nn_out_name, cam._N_ANCHORS, cam.nn_kpts, cam.nn_classes = None, 1, 0, 80
    cam._decode_mode = ('x', False)
    cam.nn_conf, cam.nn_iou, cam.nn_masks, cam.nn_w, cam.nn_h = 0.5, 0.5, 0, 640, 480
    cam.labels = oakd_driver._COCO80
    cam.raw_person_log_conf = None
    cam._latest_raw_people = []
    cam._read_layer = lambda nndata, name: np.zeros(85, np.float32)
    cam._decode = lambda raw, mode: (np.array(boxes, np.float32), np.array(scores, np.float32),
                                     np.zeros(len(scores), int), None)
    cam.get_raw_depth_frame = lambda: None
    cam._locate = lambda *a, **k: None

    class Pkt:
        def getTimestamp(self):
            class T:
                def total_seconds(self):
                    return 1.0
            return T()
    return cam, Pkt()


# two people side by side (IoU ~0.6, NMS drops the weaker) and one faint person
BOXES = [[0.40, 0.5, 0.2, 0.8], [0.45, 0.5, 0.2, 0.8], [0.8, 0.5, 0.1, 0.4]]
SCORES = [0.9, 0.7, 0.3]


def test_live_detections_unchanged_by_raw_logging():
    cam, pkt = _cam(SCORES, BOXES)
    cam._decode_nn(pkt)
    plain = cam.get_detections_with_masks()[0]
    assert cam.get_raw_people() == []
    cam.raw_person_log_conf = 0.2
    cam._decode_nn(pkt)
    logged = cam.get_detections_with_masks()[0]
    assert [d['bbox'] for d in logged] == [d['bbox'] for d in plain]
    assert len(plain) == 1                       # NMS kept one of the pair; 0.3 below conf
    raw = cam.get_raw_people()
    assert len(raw) == 3 and sorted(r[4] for r in raw) == [0.3, 0.7, 0.9]


def test_raw_logged_even_when_nothing_passes_the_cut():
    cam, pkt = _cam([0.3], [BOXES[2]])
    cam.raw_person_log_conf = 0.2
    cam._decode_nn(pkt)
    assert cam.get_detections_with_masks()[0] == [] and len(cam.get_raw_people()) == 1


class FakeOak:
    def __init__(self):
        self.t0 = time.monotonic()
        self.depth = np.full((640, 480), 2.0, np.float32)
        self.raw_person_log_conf = None
        self.nn_conf, self.nn_iou = 0.5, 0.5

    def get_depth_intrinsics(self):
        return (400.0, 400.0, 240.0, 320.0, 480, 640)

    def get_raw_depth_frame(self):
        return self.depth

    @property
    def _latest_detections_t(self):
        return round((time.monotonic() - self.t0) * 15) / 15 + self.t0

    def get_detection_observation(self):
        return [{'label': 'person', 'conf': 0.8, 'bbox': [180, 100, 300, 600]}], self._latest_detections_t

    def get_raw_people(self):
        return [[180, 100, 300, 600, 0.8], [300, 100, 400, 600, 0.3]] if self.raw_person_log_conf else []


def test_flag_starts_and_stops_recording(tmp_path, monkeypatch):
    flag = tmp_path / 'flag'
    monkeypatch.setattr(c3_live, 'RECORD_FLAG', str(flag))
    oak = FakeOak()
    trk = c3_live.C3Live(oak, lambda: {'x': 0.0, 'y': 0.0, 'theta': 0.0}, mount_x=0.1)
    trk.start()
    time.sleep(0.3)
    flag.write_text(str(tmp_path / 'rec'))
    time.sleep(1.8)
    assert oak.raw_person_log_conf == c3_live.RECORD_RAW_CONF
    flag.unlink()
    time.sleep(1.3)
    assert oak.raw_person_log_conf is None
    trk.stop()
    pk = [json.loads(line) for line in open(tmp_path / 'rec' / 'c3.ndjson')]
    assert len(pk) > 5 and all(p['k'] == 'pkt' for p in pk)
    p = pk[-1]
    assert len(p['raw']) == 2 and p['dets'][0]['conf'] == 0.8 and len(p['meas']) == 1
    assert any(t['confirmed'] for t in p['tracks']) and p['nn_iou'] == 0.5
    depth = c3_live.read_depth_log(tmp_path / 'rec' / 'c3_depth.bin')
    assert len(depth) == sum(q['depth_rec'] for q in pk)
    t, step, d = depth[0]
    assert step == 2 and d.shape == (320, 240) and np.allclose(d, 2.0)
