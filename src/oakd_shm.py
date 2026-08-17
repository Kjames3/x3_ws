"""Shared-memory transport between the x3_server parent and the OAK-D child process.

Single-writer / multi-reader seqlock over a named POSIX shared-memory block. The
writer bumps `seq` to odd before touching the payload and back to even when done;
a reader that sees an odd seq, or a changed seq across the copy, retries. This
keeps the parent lock-free -- no queue, no drain thread, so the parent never burns
GIL time on the capture path.

Header layout (64 B):
    0  seq      uint64      odd while a write is in flight
    8  nbytes   uint64      payload length
   16  ndim     int32       0 => opaque bytes (pickle), <0 => empty/cleared
   20  shape    int32 x 4
   36  dtype    16 B ascii, null padded
   52  stamp    float64     writer's time.monotonic() at publish
"""
import struct
import time

import numpy as np
from multiprocessing import shared_memory

HDR_SIZE = 64
_MAX_DIMS = 4
_RETRIES = 8

# Capacities are generous so a pipeline/resolution change never overruns a slot.
CAP_DEPTH = 4 * 1024 * 1024      # 480x640 float32 = 1.2 MB
CAP_MONO = 2 * 1024 * 1024       # 640x400 uint8 = 256 kB
CAP_META = 256 * 1024            # pickled imu/detections/status

CAPACITIES = {"depth": CAP_DEPTH, "left": CAP_MONO, "right": CAP_MONO, "meta": CAP_META}


def slot_names(prefix):
    return {k: f"{prefix}_{k}" for k in CAPACITIES}


class Slot:
    """One seqlock-protected shared-memory block."""

    def __init__(self, name, capacity=None, create=False):
        if create:
            # A stale block from a crashed run would silently alias; clear it first.
            try:
                old = shared_memory.SharedMemory(name=name)
                old.close()
                old.unlink()
            except FileNotFoundError:
                pass
            self.shm = shared_memory.SharedMemory(
                name=name, create=True, size=HDR_SIZE + capacity)
            self.shm.buf[:HDR_SIZE] = b"\x00" * HDR_SIZE
        else:
            self.shm = shared_memory.SharedMemory(name=name)
        self.name = name
        self.buf = self.shm.buf
        self.capacity = self.shm.size - HDR_SIZE
        self._created = create

    # ---------------------------------------------------------------- writer
    def _begin(self):
        seq = struct.unpack_from("<Q", self.buf, 0)[0]
        seq += 1 if seq % 2 == 0 else 2      # make it odd
        struct.pack_into("<Q", self.buf, 0, seq)
        return seq

    def _end(self, seq):
        struct.pack_into("<Q", self.buf, 0, seq + 1)   # back to even

    def write_array(self, arr):
        arr = np.ascontiguousarray(arr)
        if arr.nbytes > self.capacity:
            raise ValueError(
                f"{self.name}: {arr.nbytes} B exceeds slot capacity {self.capacity} B")
        if arr.ndim > _MAX_DIMS:
            raise ValueError(f"{self.name}: rank {arr.ndim} unsupported")
        seq = self._begin()
        shape = list(arr.shape) + [0] * (_MAX_DIMS - arr.ndim)
        struct.pack_into("<Qi", self.buf, 8, arr.nbytes, arr.ndim)
        struct.pack_into("<iiii", self.buf, 20, *shape[:_MAX_DIMS])
        self.buf[36:52] = arr.dtype.str.encode()[:16].ljust(16, b"\x00")
        struct.pack_into("<d", self.buf, 52, time.monotonic())
        view = np.ndarray(arr.shape, dtype=arr.dtype, buffer=self.buf, offset=HDR_SIZE)
        np.copyto(view, arr)
        self._end(seq)

    def write_bytes(self, payload):
        if len(payload) > self.capacity:
            raise ValueError(
                f"{self.name}: {len(payload)} B exceeds slot capacity {self.capacity} B")
        seq = self._begin()
        struct.pack_into("<Qi", self.buf, 8, len(payload), 0)
        struct.pack_into("<d", self.buf, 52, time.monotonic())
        self.buf[HDR_SIZE:HDR_SIZE + len(payload)] = payload
        self._end(seq)

    def clear(self):
        """Publish an empty payload (used when a stream stops, e.g. economy mode)."""
        seq = self._begin()
        struct.pack_into("<Qi", self.buf, 8, 0, -1)
        self._end(seq)

    # ---------------------------------------------------------------- reader
    def sequence(self):
        return struct.unpack_from("<Q", self.buf, 0)[0]

    def read_array(self):
        """(array_copy, seq, stamp) or None. The copy is mandatory: the writer may
        overwrite the block at any moment, so a view would tear."""
        for _ in range(_RETRIES):
            s1 = struct.unpack_from("<Q", self.buf, 0)[0]
            if s1 == 0 or s1 % 2:            # never written, or write in flight
                time.sleep(0.0005)
                continue
            nbytes, ndim = struct.unpack_from("<Qi", self.buf, 8)
            if ndim <= 0:
                return None
            shape = struct.unpack_from("<iiii", self.buf, 20)[:ndim]
            dtype = np.dtype(self.buf[36:52].tobytes().rstrip(b"\x00").decode())
            stamp = struct.unpack_from("<d", self.buf, 52)[0]
            if nbytes != int(np.prod(shape)) * dtype.itemsize or nbytes > self.capacity:
                time.sleep(0.0005)
                continue
            view = np.ndarray(tuple(shape), dtype=dtype, buffer=self.buf, offset=HDR_SIZE)
            out = view.copy()
            if struct.unpack_from("<Q", self.buf, 0)[0] == s1:
                return out, s1, stamp
        return None

    def read_bytes(self):
        for _ in range(_RETRIES):
            s1 = struct.unpack_from("<Q", self.buf, 0)[0]
            if s1 == 0 or s1 % 2:
                time.sleep(0.0005)
                continue
            nbytes, ndim = struct.unpack_from("<Qi", self.buf, 8)
            if ndim != 0 or nbytes > self.capacity:
                return None
            out = bytes(self.buf[HDR_SIZE:HDR_SIZE + nbytes])
            if struct.unpack_from("<Q", self.buf, 0)[0] == s1:
                return out, s1
        return None

    # ---------------------------------------------------------------- teardown
    def close(self):
        try:
            self.buf = None
            self.shm.close()
        except Exception:
            pass

    def unlink(self):
        try:
            self.shm.unlink()
        except Exception:
            pass
