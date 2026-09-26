"""C1 integrity: duplicates cannot refresh cache; wrap/reset/gaps remain explicit."""
import sys
from pathlib import Path
import threading
import copy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from tof_observation import ToFObservations
from c1_tof_audit import summarize


def frame(seq=1, tick=100, sensor='upper'):
    return dict(v=1, type='frame', sensor=sensor, seq=seq, t_ms=tick, read_us=14650,
                distance_mm=[700] * 64, target_status=[5] * 64, nb_target_detected=[1] * 64)


def test_duplicate_does_not_advance_source_clock():
    tracker = ToFObservations('test'); tracker.connect()
    a = tracker.envelope(frame(), 1000000000, 2000000000)
    b = tracker.envelope(frame(), 9000000000, 10000000000)
    c = tracker.envelope(frame(2, 166), 9100000000, 10100000000)
    assert a['accepted'] and not b['accepted']
    assert b['continuity'] == 'duplicate'
    assert c['source_interval_ms'] == 66
    assert a['clock']['acquisition_time_ns'] is None
    assert not a['clock']['synchronized']


def test_millis_and_sequence_wrap_are_not_reboots():
    tracker = ToFObservations('test'); tracker.connect()
    tracker.envelope(frame(0xffffffff, 0xfffffff0), 1, 1)
    row = tracker.envelope(frame(0, 50), 2, 2)
    assert row['source_interval_ms'] == 66
    assert row['source_read_complete_ms_unwrapped'] == 0x100000032
    assert row['sensor_epoch'] == 0 and row['accepted']


def test_reset_quarantines_first_frame_and_separates_epoch():
    tracker = ToFObservations('test'); tracker.connect()
    tracker.envelope(frame(100, 9999), 1, 1)
    reset = tracker.envelope(frame(1, 100), 2, 2)
    next_row = tracker.envelope(frame(2, 166), 3, 3)
    assert reset['continuity'] == 'reset_or_reorder' and not reset['accepted']
    assert reset['sensor_epoch'] == next_row['sensor_epoch'] == 1
    assert next_row['accepted']
    tracker.connect()
    row = tracker.envelope(frame(3, 232), 4, 4)
    assert row['connection_id'] == 1 and row['continuity'] == 'first'


def test_sensor_identity_gaps_and_payload_are_preserved():
    tracker = ToFObservations('test'); tracker.connect()
    original = frame()
    row = tracker.envelope(original, 1, 1)
    original['distance_mm'][0] = 0
    assert row['payload']['distance_mm'][0] == 700
    tracker.envelope(frame(1, 101, 'lower'), 1, 1)
    row = tracker.envelope(frame(4, 298), 2, 2)
    assert row['missing_sequence_count'] == 2
    assert row['source_interval_ms'] == 198


def test_replay_is_deterministic_and_exposes_recording_loss():
    tracker = ToFObservations('test'); tracker.connect()
    a = tracker.envelope(frame(1, 100), 1000000000, 1000000000)
    tracker.envelope(frame(2, 166), 1066000000, 1066000000)  # lost by recorder
    b = tracker.envelope(frame(3, 232), 1232000000, 1232000000)
    summary = summarize([a, b])
    assert summary == summarize(copy.deepcopy([a, b]))
    assert summary['streams'][0]['recorded_sequence_gaps'] == 1
    assert summary['streams'][0]['missing_sequences'] == 0
    assert not summary['c1_complete']


def test_real_reader_duplicate_does_not_refresh_cache_or_publish_cloud():
    from drivers_x3 import TeensyToFArrays
    reader = TeensyToFArrays.__new__(TeensyToFArrays)
    from tof_clock import TeensyClock
    reader._clock_cls = TeensyClock; reader._source_clock = TeensyClock()
    reader._observations = ToFObservations('test'); reader._observations.connect()
    reader._lock = threading.Lock(); reader._frames = {}; reader._metadata = {}
    reader._active = {}; published = []; evidence = []
    reader.on_frame = lambda *args: published.append(args)
    reader.on_observation = evidence.append
    reader._handle(frame(), 1000000000, 2000000000)
    reader._handle(frame(), 9000000000, 10000000000)
    assert len(published) == 1 and len(evidence) == 2
    assert reader._frames['upper'][3] == 1.0
    assert reader.latest_observation('upper')['receipt_monotonic_ns'] == 1000000000
    assert 'nb_target_detected' in evidence[0]['payload']
