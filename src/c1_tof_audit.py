#!/usr/bin/env python3
"""Record /tof/observations without opening the Teensy's owned serial port.

Record: python3 src/c1_tof_audit.py --output /tmp/c1-tof --seconds 30
Replay: python3 src/c1_tof_audit.py --replay /tmp/c1-tof/observations.jsonl
Replay is deterministic and never substitutes playback time for observation time.
This is the ToF C1 evidence stage, not synchronized RGB-D/pose qualification.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time

from teensy_tof_serial import parse_record


def summarize(records):
    streams = {}
    bad = 0
    total = 0
    last_receipt = {}
    last_seq = {}
    for row in records:
        total += 1
        try:
            if row['schema'] != 'x3.tof.observation.v1':
                raise ValueError('schema')
            rec = parse_record(json.dumps(row['payload']))
            if row['sensor'] != rec['sensor']:
                raise ValueError('identity mismatch')
            if type(row['receipt_monotonic_ns']) is not int or row['receipt_monotonic_ns'] < 0:
                raise ValueError('receipt clock')
            if rec['type'] != 'frame':
                continue
            key = (row['session_id'], row['connection_id'], row['sensor'], row['sensor_epoch'])
            accepted = row['accepted']
            if type(accepted) is not bool:
                raise ValueError('accepted')
            if not isinstance(row['clock'], dict):
                raise ValueError('clock metadata')
            if not isinstance(row['continuity'], str):
                raise ValueError('continuity')
            if row['source_interval_ms'] is not None and type(row['source_interval_ms']) is not int:
                raise ValueError('source interval')
            hash(key)
            if type(row['missing_sequence_count']) is not int or row['missing_sequence_count'] < 0:
                raise ValueError('sequence gap')
        except (KeyError, TypeError, ValueError):
            bad += 1
            continue
        stats = streams.setdefault(key, dict(frames=0, accepted=0, missing=0, continuity=Counter(),
                                             first_ns=None, last_ns=None, max_receipt_gap_ms=0,
                                             receipt_gaps_over_500ms=0, receipt_reversals=0,
                                             source_intervals=[], read_us=[], synchronized=0, clock_width_ms=[], recorded_sequence_gaps=0))
        stats['frames'] += 1
        stats['continuity'][row['continuity']] += 1
        stats['missing'] += row['missing_sequence_count']
        stats['synchronized'] += bool(row['clock'].get('synchronized'))
        interval = row['clock'].get('read_complete_interval_ns')
        if row['clock'].get('synchronized') and isinstance(interval, list) and len(interval) == 2:
            stats['clock_width_ms'].append((interval[1] - interval[0]) / 1e6)
        if accepted:
            if key in last_seq:
                delta = (rec['seq'] - last_seq[key]) & 0xffffffff
                if 1 < delta < 0x80000000:
                    stats['recorded_sequence_gaps'] += delta - 1
            last_seq[key] = rec['seq']
            now = row['receipt_monotonic_ns']
            if key in last_receipt:
                gap = (now - last_receipt[key]) / 1e6
                stats['max_receipt_gap_ms'] = max(stats['max_receipt_gap_ms'], gap)
                stats['receipt_gaps_over_500ms'] += gap > 500
                stats['receipt_reversals'] += gap < 0
            last_receipt[key] = now
            if stats['first_ns'] is None:
                stats['first_ns'] = now
            stats['last_ns'] = now
            stats['accepted'] += 1
            stats['read_us'].append(rec['read_us'])
            # Source intervals across seq gaps span more than one frame; retain separately.
            if row['source_interval_ms'] is not None:
                stats['source_intervals'].append(row['source_interval_ms'])
    result = []
    for key, s in sorted(streams.items()):
        duration = (s['last_ns'] - s['first_ns']) / 1e9 if s['accepted'] > 1 else 0
        result.append(dict(session_id=key[0], connection_id=key[1], sensor=key[2], epoch=key[3],
                           frames=s['frames'], accepted=s['accepted'],
                           received_fps=(s['accepted'] - 1) / duration if duration > 0 else None,
                           missing_sequences=s['missing'],
                           recorded_sequence_gaps=s['recorded_sequence_gaps'], continuity=dict(s['continuity']),
                           max_receipt_gap_ms=s['max_receipt_gap_ms'],
                           receipt_gaps_over_500ms=s['receipt_gaps_over_500ms'],
                           receipt_reversals=s['receipt_reversals'],
                           mean_read_ms=sum(s['read_us']) / len(s['read_us']) / 1000 if s['read_us'] else None,
                           max_source_interval_ms=max(s['source_intervals'], default=None),
                           synchronized_frames=s['synchronized'],
                           read_complete_interval_width_ms_max=max(s['clock_width_ms'], default=None)))
    return {'schema': 'x3.c1.tof.audit.v1', 'records': total, 'invalid_records': bad,
            'streams': result, 'acquisition_age_available': False,
            'clock_error_bound_available': False,
            'conditional_read_complete_intervals_available': any(s['clock_width_ms'] for s in streams.values()),
            'c1_complete': False,
            'limitations': ['Receipt age is not acquisition age.',
                            'Read-completion intervals require sync-capable firmware and stated reset/drift assumptions; exposure time remains unknown.',
                            'RGB-depth pairing and pose alignment require separate validation.']}


def load_records(path):
    with open(path) as source:
        for number, line in enumerate(source, 1):
            try:
                yield json.loads(line)
            except ValueError as exc:
                raise ValueError(f'{path}:{number}: malformed JSON') from exc


def freeze_manifest(directory):
    root = Path(__file__).resolve().parents[1]
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args], text=True)
    (directory / 'source.diff').write_text(git('diff', 'HEAD', '--', 'src', 'scripts', 'config'))
    files = ['src/tof_observation.py', 'src/c1_tof_audit.py', 'src/teensy_tof_serial.py',
             'src/drivers_x3.py', 'src/server_x3.py', 'scripts/teensy_tof_test/teensy_tof_test.ino',
             'src/yahboomcar_description/urdf/yahboomcar_X3.urdf', 'config/tof_floor_baseline.json']
    hashes = {}
    for name in files:
        data = (root / name).read_bytes()
        target = directory / 'snapshot' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    manifest = dict(schema='x3.c1.capture.v1', git_head=git('rev-parse', 'HEAD').strip(),
                    git_status=git('status', '--short'), files_sha256=hashes,
                    started_unix_ns=time.time_ns(), started_monotonic_ns=time.monotonic_ns(),
                    source_host='recorder_host_not_verified_publisher',
                    topic='/tof/observations', synchronized=False)
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output', type=Path, help='new capture directory; run on the Jetson')
    mode.add_argument('--replay', type=Path, help='recorded observations.jsonl')
    p.add_argument('--seconds', type=float, default=30)
    args = p.parse_args()
    if args.replay:
        print(json.dumps(summarize(load_records(args.replay)), indent=2))
        return 0
    if args.seconds <= 0:
        p.error('--seconds must be positive')
    # Lazy imports keep replay usable on a laptop without ROS installed.
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('c1_tof_audit')
    args.output.mkdir(parents=True, exist_ok=False)
    freeze_manifest(args.output)
    path = args.output / 'observations.jsonl'
    malformed = 0
    with path.open('x') as output:
        def receive(msg):
            nonlocal malformed
            try:
                row = json.loads(msg.data)
                if not isinstance(row, dict):
                    raise ValueError('object required')
                row['recorder_receipt_monotonic_ns'] = time.monotonic_ns()
                row['recorder_receipt_unix_ns'] = time.time_ns()
                output.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
            except (ValueError, TypeError):
                malformed += 1
        sub = node.create_subscription(String, '/tof/observations', receive, 100)
        print(f'CAPTURE START: {args.seconds:g} s -> {args.output}', flush=True)
        deadline = time.monotonic() + args.seconds
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline - time.monotonic())))
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
    report = summarize(load_records(path))
    report['malformed_topic_messages'] = malformed
    report['observations_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('CAPTURE END\n' + json.dumps(report, indent=2))
    return 0 if {s['sensor'] for s in report['streams'] if s['accepted']} == {'upper', 'lower'} and not report['invalid_records'] and not malformed else 2


if __name__ == '__main__':
    raise SystemExit(main())
