"""Read SiT dataset clips: PCD clouds (binary_compressed = LZF), 3D labels, ego poses.

No open3d/pypcd needed.  The LZF decoder is plain Python (~0.3 s per 25k-point frame);
callers parallelise over frames.

Frames (verified by points-in-box counts, see sit_person_dataset.py --check-frames):
label_3d rows are 'class id h w l x y z yaw' in the WORLD frame (the same trap the
velocity converter hit, project_sic_dataset memory); ego_trajectory/<i>.txt is the
row-major 4x4 T_world_ego, comma-separated; the clouds are in the ego (lidar) frame.
"""
import struct

import numpy as np

_PCD_TYPES = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2',
              ('U', 4): '<u4', ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4'}


def lzf_decompress(src, out_len):
    src = memoryview(src)
    out = bytearray(out_len)
    i = o = 0
    n = len(src)
    while i < n:
        ctrl = src[i]
        i += 1
        if ctrl < 32:                       # literal run of ctrl + 1 bytes
            ln = ctrl + 1
            out[o:o + ln] = src[i:i + ln]
            i += ln
            o += ln
        else:                               # back reference
            ln = ctrl >> 5
            ref = o - ((ctrl & 0x1f) << 8) - 1
            if ln == 7:
                ln += src[i]
                i += 1
            ref -= src[i]
            i += 1
            ln += 2
            if ref + ln <= o:
                out[o:o + ln] = out[ref:ref + ln]
                o += ln
            else:                           # overlapping copy, byte by byte
                for _ in range(ln):
                    out[o] = out[ref]
                    o += 1
                    ref += 1
    if o != out_len:
        raise ValueError(f'LZF: got {o} bytes, expected {out_len}')
    return bytes(out)


def read_pcd(path):
    """-> dict field name -> numpy array."""
    with open(path, 'rb') as f:
        raw = f.read()
    header, pos = {}, 0
    while True:
        end = raw.index(b'\n', pos)
        line = raw[pos:end].decode('ascii').strip()
        pos = end + 1
        if not line or line.startswith('#'):
            continue
        key, *vals = line.split()
        header[key] = vals
        if key == 'DATA':
            break
    names, sizes, types = header['FIELDS'], [int(s) for s in header['SIZE']], header['TYPE']
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(names))]
    n = int(header['POINTS'][0])
    dts = [_PCD_TYPES[(t, s)] for t, s in zip(types, sizes)]
    mode = header['DATA'][0]
    if mode == 'binary_compressed':
        csize, usize = struct.unpack_from('<II', raw, pos)
        data = lzf_decompress(raw[pos + 8:pos + 8 + csize], usize)
        out, off = {}, 0
        for name, dt, s, c in zip(names, dts, sizes, counts):   # column-major
            out[name] = np.frombuffer(data, dtype=dt, count=n * c, offset=off)
            off += n * s * c
        return out
    if mode == 'binary':
        rec = np.dtype([(nm, dt) for nm, dt in zip(names, dts)])
        arr = np.frombuffer(raw, dtype=rec, count=n, offset=pos)
        return {nm: arr[nm] for nm in names}
    arr = np.loadtxt(raw[pos:].decode('ascii').splitlines(), ndmin=2)
    return {nm: arr[:, i] for i, nm in enumerate(names)}


def read_labels(path):
    """-> list of (cls, id, h, w, l, x, y, z, yaw)."""
    out = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 9:
                continue
            out.append((p[0], p[1], *map(float, p[2:9])))
    return out


def read_pose(path):
    with open(path) as f:
        v = [float(t) for t in f.read().replace(',', ' ').split()]
    return np.array(v[:16]).reshape(4, 4)
