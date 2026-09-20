"""USB framing must recover from partial/oversize input without mixing sensors."""
import importlib.util
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('teensy_tof_serial', Path(__file__).parents[1] / 'src/teensy_tof_serial.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def frame(sensor='upper'):
    return dict(v=1, type='frame', sensor=sensor, seq=12, t_ms=1234, read_us=14650,
                distance_mm=[700] * 64, target_status=[5] * 64, nb_target_detected=[1] * 64)


class ProtocolTest(unittest.TestCase):
    def test_fragmented_and_concatenated_frames(self):
        upper, lower = frame(), frame('lower')
        stream = (json.dumps(upper) + '\n' + json.dumps(lower) + '\n').encode()
        decoder = m.LineDecoder()
        records = []
        for start in range(0, len(stream), 17):
            records.extend(decoder.feed(stream[start:start + 17]))
        self.assertEqual(records, [upper, lower])
        self.assertEqual(decoder.rejected, 0)

    def test_resynchronizes_after_garbage_and_oversize(self):
        decoder = m.LineDecoder()
        valid = frame()
        records = decoder.feed(b'boot text\n' + b'x' * (m.MAX_LINE + 10) + b'\n' + json.dumps(valid).encode() + b'\n')
        self.assertEqual(records, [valid])
        self.assertEqual(decoder.rejected, 2)
        self.assertEqual(len(decoder.pending), 0)

    def test_rejects_invalid_schema(self):
        for name, value in [('distance_mm', [1] * 63), ('target_status', [256] * 64),
                            ('seq', True), ('sensor', 'other'), ('v', 2)]:
            with self.subTest(name=name):
                rec = frame(); rec[name] = value
                with self.assertRaises(ValueError):
                    m.parse_record(json.dumps(rec))

    def test_retains_invalid_zones_for_downstream_filtering(self):
        rec = frame(); rec['distance_mm'][0] = -1
        rec['target_status'][0] = 255; rec['nb_target_detected'][0] = 0
        self.assertEqual(m.parse_record(json.dumps(rec)), rec)


if __name__ == '__main__':
    unittest.main()
