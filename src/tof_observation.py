"""C1 ToF evidence envelopes. No inferred acquisition timestamps or clock sync.

Sequence/time reversals are ambiguous (reset or reordered old input); start a
new per-sensor segment and quarantine its first frame. Never silently turn a
cached/duplicate frame into new evidence. The MCU has no boot identifier.
"""
import copy
import uuid

UINT32_MASK = (1 << 32) - 1
HALF_UINT32 = 1 << 31


class ToFObservations:
    def __init__(self, session_id=None):
        self.session_id = session_id or str(uuid.uuid4())
        self.connection = -1
        self.previous = {}
        self.epochs = {}

    def connect(self):
        self.connection += 1
        self.previous.clear()
        self.epochs.clear()

    def envelope(self, record, receipt_monotonic_ns, receipt_unix_ns):
        name = record['sensor']
        out = {
            'schema': 'x3.tof.observation.v1',
            'session_id': self.session_id,
            'connection_id': self.connection,
            'sensor': name,
            'frame_id': f'tof_{name}_link' if name != 'teensy' else None,
            'receipt_monotonic_ns': receipt_monotonic_ns,
            'receipt_unix_ns': receipt_unix_ns,
            'receipt_kind': 'serial_chunk_read_complete',
            'clock': {
                'source': 'teensy_millis_uint32',
                'source_stamp_kind': 'i2c_read_complete',
                'acquisition_time_ns': None,
                'source_to_host_offset_ns': None,
                'synchronization_uncertainty_ns': None,
                'synchronized': False,
            },
            'payload': copy.deepcopy(record),
            'accepted': False,
            'continuity': 'non_frame',
            'missing_sequence_count': 0,
        }
        if record['type'] != 'frame':
            return out
        seq, tick = record['seq'], record['t_ms']
        prev = self.previous.get(name)
        epoch = self.epochs.get(name, 0)
        unwrapped = tick
        continuity = 'first'
        accepted = True
        interval = None
        missing = 0
        if prev is not None:
            ds = (seq - prev['seq']) & UINT32_MASK
            dt = (tick - prev['tick']) & UINT32_MASK
            if ds == 0:
                continuity, accepted = 'duplicate', False
                unwrapped = None
            elif ds >= HALF_UINT32 or dt >= HALF_UINT32:
                # Cannot distinguish reboot from reordered records with v1 MCU protocol.
                epoch += 1
                continuity, accepted = 'reset_or_reorder', False
            else:
                continuity = 'gap' if ds > 1 else 'continuous'
                missing = ds - 1
                interval = dt
                unwrapped = prev['unwrapped'] + dt
        if continuity != 'duplicate':
            self.previous[name] = {'seq': seq, 'tick': tick, 'unwrapped': unwrapped}
            self.epochs[name] = epoch
        out.update(accepted=accepted, continuity=continuity,
                   missing_sequence_count=missing, sensor_epoch=epoch,
                   source_read_complete_ms_unwrapped=unwrapped,
                   source_interval_ms=interval,
                   observation_id=f'{self.session_id}/{self.connection}/{name}/{epoch}/{seq}')
        return out
