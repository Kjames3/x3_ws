#!/usr/bin/env python3
"""Validate/record dual Teensy ToF NDJSON. No ROS or motor commands.

python3 src/teensy_tof_serial.py --port /dev/ttyACM0 --seconds 30 --output tof.jsonl
Close Arduino Serial Monitor first. USB timestamps are read-completion times
on the MCU (32-bit milliseconds), not synchronized ROS acquisition timestamps.
"""
import argparse
from collections import Counter
import json
import sys
import time

MAX_LINE = 4096


def parse_record(line):
    """Return a validated record, raising ValueError for malformed input."""
    if len(line) > MAX_LINE:
        raise ValueError('record too long')
    try:
        record = json.loads(line)
    except (ValueError, UnicodeError) as exc:
        raise ValueError('invalid JSON') from exc
    if not isinstance(record, dict) or type(record.get('v')) is not int or record['v'] != 1:
        raise ValueError('unsupported protocol version')
    if record.get('type') == 'sync':
        if record.get('sensor') != 'teensy':
            raise ValueError('sync source')
        for name in ('token', 'rx_ms', 'tx_ms'):
            if type(record.get(name)) is not int or not 0 <= record[name] <= 0xffffffff:
                raise ValueError('invalid sync field ' + name)
        return record
    if record.get('sensor') not in ('upper', 'lower'):
        raise ValueError('unknown sensor')
    if record.get('type') not in ('frame', 'stats', 'status'):
        raise ValueError('unknown record type')
    if record['type'] == 'frame':
        for name in ('seq', 't_ms', 'read_us'):
            value = record.get(name)
            if type(value) is not int or not 0 <= value <= 0xffffffff:
                raise ValueError(f'invalid {name}')
        for name, lo, hi in (('distance_mm', -32768, 32767),
                             ('target_status', 0, 255), ('nb_target_detected', 0, 255)):
            values = record.get(name)
            if not isinstance(values, list) or len(values) != 64:
                raise ValueError(f'{name} must have 64 zones')
            if any(type(v) is not int or not lo <= v <= hi for v in values):
                raise ValueError(f'invalid {name} value')
    return record


class LineDecoder:
    """Bounded incremental framing; discard oversize records through newline."""
    def __init__(self):
        self.pending = bytearray()
        self.discarding = False
        self.rejected = 0

    def feed(self, chunk):
        records = []
        for byte in chunk:
            if byte == 10:
                if self.discarding:
                    self.discarding = False
                elif self.pending.strip():
                    try:
                        records.append(parse_record(self.pending))
                    except ValueError:
                        self.rejected += 1
                self.pending.clear()
            elif not self.discarding:
                self.pending.append(byte)
                if len(self.pending) > MAX_LINE:
                    self.pending.clear()
                    self.discarding = True
                    self.rejected += 1
        return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='/dev/serial/by-id/... preferred')
    parser.add_argument('--seconds', type=float, default=0, help='0 runs until Ctrl-C')
    parser.add_argument('--output', help='new JSONL recording file; existing files are not overwritten')
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error('--seconds must be nonnegative')
    try:
        import serial
    except ImportError:
        parser.exit(1, 'Install pyserial: python3 -m pip install pyserial\n')
    decoder = LineDecoder()
    count = Counter()
    previous = {}
    gaps = Counter()
    resets = Counter()
    last_seen = {}
    started = report_at = time.monotonic()
    output = None
    try:
        if args.output:
            output = open(args.output, 'x', buffering=1)
        with serial.Serial(args.port, 115200, timeout=0.2, write_timeout=1, exclusive=True) as port:
            port.write(b'r')  # Enable raw frames if previous monitor selected benchmark mode.
            while not args.seconds or time.monotonic() - started < args.seconds:
                for rec in decoder.feed(port.read(min(max(port.in_waiting, 1), MAX_LINE))):
                    if output:
                        output.write(json.dumps(rec, separators=(',', ':')) + '\n')
                    name = rec['sensor']
                    if rec['type'] == 'frame':
                        seq = rec['seq']
                        if name in previous:
                            delta = (seq - previous[name]) & 0xffffffff
                            if delta == 0 or delta >= 0x80000000:
                                resets[name] += 1
                            elif delta > 1:
                                gaps[name] += delta - 1
                        previous[name] = seq
                        count[name] += 1
                        last_seen[name] = time.monotonic()
                    else:
                        print(json.dumps(rec), file=sys.stderr)
                now = time.monotonic()
                if now - report_at >= 5:
                    for name in ('upper', 'lower'):
                        age = f'{now - last_seen[name]:.1f}s' if name in last_seen else 'never'
                        print(f'{name}: received_fps={count[name] / (now - report_at):.2f} '
                              f'sequence_gaps={gaps[name]} resets_or_duplicates={resets[name]} '
                              f'last_frame_age={age}', file=sys.stderr)
                    print(f'rejected_records={decoder.rejected}', file=sys.stderr)
                    count.clear()
                    report_at = now
    except KeyboardInterrupt:
        pass
    except (OSError, serial.SerialException) as exc:
        print(f'Teensy reader: {exc}', file=sys.stderr)
        return 1
    finally:
        if output:
            output.close()
    missing = {'upper', 'lower'} - last_seen.keys()
    if missing:
        print('No valid frames received from: ' + ', '.join(sorted(missing)), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
