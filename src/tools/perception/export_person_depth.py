#!/usr/bin/env python3
"""Lossless depth-only replay bundle, retaining original pair-index provenance.

Read-only against rosbag SQLite. Each image is checked against its recorded
SHA-256 and acquisition timestamp. Cached detections/masks and pair-index.json
are copied separately; RGB and unrelated ROS topics are intentionally omitted.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import zipfile

import numpy as np


def export(run):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    rows=json.loads((run/'pair-index.json').read_text())
    dest=run/'depth-frames.npz'
    tmp=run/'depth-frames.npz.tmp'
    conns={}; hashes=[]
    try:
        with zipfile.ZipFile(tmp,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
            for i,row in enumerate(rows):
                ref=row['depth']
                if ref['db'] not in conns:
                    conns[ref['db']]=sqlite3.connect(f"file:{run/'bag'/ref['db']}?mode=ro",uri=True)
                payload=conns[ref['db']].execute('select data from messages where id=?',(ref['message_id'],)).fetchone()
                if payload is None: raise ValueError('Missing depth message')
                msg=deserialize_message(payload[0],Image); raw=bytes(msg.data)
                digest=hashlib.sha256(raw).hexdigest()
                if digest!=ref['sha256']: raise ValueError(f'Depth hash mismatch at {i}')
                if msg.header.stamp.sec*10**9+msg.header.stamp.nanosec!=ref['stamp_ns']:
                    raise ValueError(f'Depth timestamp mismatch at {i}')
                arr=np.frombuffer(raw,dtype='<u2').reshape(ref['height'],ref['width'])
                with z.open(f'frame_{i}.npy','w') as f:
                    np.lib.format.write_array(f,arr,allow_pickle=False)
                hashes.append(digest)
        tmp.replace(dest)
    finally:
        for conn in conns.values(): conn.close()
    record=dict(format='lossless indexed depth frames; RGB not included',frames=len(rows),
                index_sha256=hashlib.sha256((run/'pair-index.json').read_bytes()).hexdigest(),
                archive_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                depth_sha256=hashes)
    (run/'depth-export.json').write_text(json.dumps(record,indent=2)+'\n')
    print(run.name,len(rows),dest.stat().st_size,flush=True)


def verify(run):
    record=json.loads((run/'depth-export.json').read_text())
    rows=json.loads((run/'pair-index.json').read_text())
    assert hashlib.sha256((run/'pair-index.json').read_bytes()).hexdigest()==record['index_sha256']
    assert hashlib.sha256((run/'depth-frames.npz').read_bytes()).hexdigest()==record['archive_sha256']
    with np.load(run/'depth-frames.npz') as data:
        assert len(data.files)==len(rows)==record['frames']
        for i,row in enumerate(rows):
            arr=data[f'frame_{i}']
            assert arr.shape==(row['depth']['height'],row['depth']['width'])
            assert hashlib.sha256(arr.tobytes()).hexdigest()==row['depth']['sha256']==record['depth_sha256'][i]
    return record['frames']


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('runs',type=Path,nargs='+')
    ap.add_argument('--verify',action='store_true')
    args=ap.parse_args()
    for run in args.runs:
        if args.verify: print(run.name,verify(run),'verified',flush=True)
        else: export(run)
