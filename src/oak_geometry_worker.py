"""Private subprocess for display-only geometry; one request, no work queue.

Only OakSurroundings' background thread may call compute. IPC never runs on the
camera, ROS executor or asyncio thread. The socket is inherited locally, not a
network listener. Pickle messages are trusted parent/child data only.
"""
import os
from pathlib import Path
import pickle
import socket
import struct
import subprocess
import sys
import time

MAX_MESSAGE = 8 * 1024 * 1024
MAX_AGE_NS = 500_000_000


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError('geometry worker deadline exceeded')
    return remaining


def _send(sock, value, deadline):
    payload = pickle.dumps(value, protocol=5)
    if len(payload) > MAX_MESSAGE:
        raise ValueError('geometry worker message exceeds byte limit')
    sock.settimeout(_remaining(deadline))
    sock.sendall(struct.pack('!I', len(payload)) + payload)


def _recv(sock, deadline):
    def read(count):
        parts = bytearray()
        while len(parts) < count:
            sock.settimeout(_remaining(deadline))
            chunk = sock.recv(count-len(parts))
            if not chunk:
                raise EOFError('geometry worker socket closed')
            parts.extend(chunk)
        return parts
    count, = struct.unpack('!I', read(4))
    if count > MAX_MESSAGE:
        raise ValueError('geometry worker message exceeds byte limit')
    return pickle.loads(read(count))


def _fresh(source):
    return 0 <= time.monotonic_ns()-source <= MAX_AGE_NS


class GeometryWorker:
    def __init__(self):
        self.socket, remote = socket.socketpair()
        self.process = None
        self.sequence = 0
        try:
            env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            self.process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), str(remote.fileno())],
                pass_fds=(remote.fileno(),), stdin=subprocess.DEVNULL, env=env)
            remote.close()
            if _recv(self.socket, time.monotonic()+3) != 'ready':
                raise RuntimeError('geometry worker handshake failed')
        except Exception:
            remote.close()
            self.close()
            raise

    def compute(self, inputs, source, timings=None):
        if not _fresh(source):
            return None
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('geometry worker exited')
        self.sequence += 1
        deadline = time.monotonic()+max(0, (MAX_AGE_NS-(time.monotonic_ns()-source))/1e9)
        try:
            _send(self.socket, (self.sequence, source, inputs, timings is not None), deadline)
            seq, outputs, measured, error = _recv(self.socket, deadline)
            if seq != self.sequence:
                raise RuntimeError('geometry worker response sequence mismatch')
            if error:
                raise RuntimeError('geometry worker: '+error)
            if not _fresh(source):
                return None
            if timings is not None:
                timings.update(measured)
            return outputs
        except Exception:
            # A partial IPC message cannot be reused. Caller may create a fresh
            # worker after cooldown; no old response can contaminate a new frame.
            self.close()
            raise

    def close(self):
        self.socket.close()
        if self.process is not None:
            try:
                self.process.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=.5)


def _child(fd):
    import cv2
    from oak_color_geometry import refine
    cv2.setNumThreads(1)
    with socket.socket(fileno=fd) as sock:
        _send(sock, 'ready', time.monotonic()+3)
        while True:
            try:
                seq, source, inputs, profile = _recv(sock, time.monotonic()+60)
                outputs, measured, error = None, {} if profile else None, None
                if _fresh(source):
                    try:
                        outputs = refine(*inputs, timings=measured)
                    except Exception as exc:
                        error = str(exc)
                if not _fresh(source):
                    outputs = None
                _send(sock, (seq, outputs, measured, error), time.monotonic()+.5)
            except (EOFError, OSError, TimeoutError):
                return


if __name__ == '__main__':
    _child(int(sys.argv[1]))
