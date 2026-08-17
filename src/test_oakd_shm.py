"""Round-trip tests for the OAK-D shared-memory transport (oakd_shm.Slot)."""
import pickle
import sys

import numpy as np

from oakd_shm import CAPACITIES, Slot, slot_names


def main():
    names = slot_names("x3test")

    w = Slot(names["depth"], capacity=CAPACITIES["depth"], create=True)
    r = Slot(names["depth"])
    a = (np.random.rand(480, 640) * 5).astype(np.float32)
    w.write_array(a)
    got = r.read_array()
    assert got is not None, "depth read failed"
    b, seq, stamp = got
    assert b.shape == (480, 640) and b.dtype == np.float32, (b.shape, b.dtype)
    assert np.array_equal(a, b), "depth payload mismatch"
    print("depth roundtrip OK (seq=%d)" % seq)

    wm = Slot(names["left"], capacity=CAPACITIES["left"], create=True)
    rm = Slot(names["left"])
    m = (np.random.rand(400, 640) * 255).astype(np.uint8)
    wm.write_array(m)
    mb, _, _ = rm.read_array()
    assert np.array_equal(m, mb), "mono payload mismatch"
    print("mono roundtrip OK")

    wm.write_array(np.zeros((400, 640, 3), np.uint8))
    cb, _, _ = rm.read_array()
    assert cb.shape == (400, 640, 3), cb.shape
    print("3-channel shape OK", cb.shape)

    wt = Slot(names["meta"], capacity=CAPACITIES["meta"], create=True)
    rt = Slot(names["meta"])
    meta = {"available": True, "detections": [{"label": "person", "x": 1.0}],
            "labels": ["a"] * 80}
    wt.write_bytes(pickle.dumps(meta))
    pb, _ = rt.read_bytes()
    assert pickle.loads(pb) == meta, "meta payload mismatch"
    print("meta roundtrip OK")

    s0 = wt.sequence()
    wt.write_bytes(b"x")
    assert wt.sequence() > s0, "sequence did not advance"
    assert wt.sequence() % 2 == 0, "sequence left odd after write"
    print("seqlock parity OK")

    w.clear()
    assert r.read_array() is None, "clear did not blank the slot"
    print("clear OK")

    # oversized payload must raise, not corrupt the block
    try:
        wt.write_bytes(b"x" * (CAPACITIES["meta"] + 1))
        raise AssertionError("oversized write should have raised")
    except ValueError:
        print("oversize guard OK")

    for s in (w, wm, wt):
        s.unlink()
    for s in (w, r, wm, rm, wt, rt):
        s.close()
    print("ALL TRANSPORT TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
