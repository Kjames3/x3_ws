#!/usr/bin/env python3
"""
extract_superblob.py — carve a plain OpenVINO/MyriadX .blob out of a Luxonis
.superblob container.

depthai 2.32 (the version pinned on the robot for the OAK-D Lite v2 API) does
NOT expose dai.OpenVINO.SuperBlob, so it cannot load a .superblob directly. A
superblob is simply:

    [ header: 17 x int64 BIG-ENDIAN ][ base blob ][ patch 0 ] ... [ patch 15 ]
      header[0]      = base blob size (bytes)
      header[1..16]  = per-shave patch sizes

The base blob is a complete, valid blob for its compiled SHAVE count (8 for
yolo26n here), which is all we need — the patches only re-target other SHAVE
counts. This script writes <name>.blob next to the <name>.superblob.

Usage:
    python3 extract_superblob.py src/blobs/yolo26n/yolo26n.superblob
"""
import struct
import sys
from pathlib import Path

HEADER_INT64_COUNT = 17  # blobSize + 16 patch sizes
HEADER_BYTES = HEADER_INT64_COUNT * 8


def extract(superblob_path: str) -> str:
    data = Path(superblob_path).read_bytes()
    if len(data) < HEADER_BYTES:
        raise ValueError("file too small to be a superblob")
    header = struct.unpack_from(">%dq" % HEADER_INT64_COUNT, data, 0)  # big-endian
    blob_size = header[0]
    patch_sizes = header[1:]
    expected = HEADER_BYTES + blob_size + sum(p for p in patch_sizes if p > 0)
    if expected != len(data):
        raise ValueError(
            f"header/size mismatch: header says {expected} bytes, file is {len(data)}. "
            "Is this really a .superblob?"
        )
    base = data[HEADER_BYTES:HEADER_BYTES + blob_size]
    out_path = str(Path(superblob_path).with_suffix(".blob"))
    Path(out_path).write_bytes(base)
    print(f"wrote {out_path} ({len(base)} bytes, base blob for its compiled SHAVE count)")
    return out_path


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    extract(sys.argv[1])
