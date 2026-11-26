import os
import signal
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from oak_geometry_worker import GeometryWorker, MAX_MESSAGE, _recv
from oak_color_geometry import refine


@pytest.fixture
def inputs():
    rgb=np.full((40,40,3),[20,40,80],np.uint8)
    depth=np.full((40,40),1000,np.uint16)
    return (rgb,depth,np.ones(depth.shape,bool),np.array([[100,0,20],[0,100,20],[0,0,1]]),.0821)


def test_real_process_exact_outputs_and_stale_rejection(inputs):
    worker=GeometryWorker()
    try:
        timings={}
        outputs=worker.compute(inputs,time.monotonic_ns(),timings)
        for a,b in zip(outputs,refine(*inputs)):
            np.testing.assert_array_equal(a,b)
        assert timings['total']['wall_ms'] >= 0
        assert worker.compute(inputs,time.monotonic_ns()-600_000_000) is None
        assert worker.compute(inputs,time.monotonic_ns()+1_000_000_000) is None
    finally:
        worker.close()
    assert worker.process.poll() is not None


def test_dead_child_and_replacement(inputs):
    worker=GeometryWorker()
    worker.process.kill();worker.process.wait()
    with pytest.raises(RuntimeError,match='exited'):
        worker.compute(inputs,time.monotonic_ns())
    worker.close()
    replacement=GeometryWorker()
    try:
        assert replacement.compute(inputs,time.monotonic_ns()) is not None
    finally:
        replacement.close()


def test_hung_child_timeout_and_cleanup(inputs):
    worker=GeometryWorker()
    os.kill(worker.process.pid,signal.SIGSTOP)
    start=time.monotonic()
    with pytest.raises((TimeoutError,OSError)):
        worker.compute(inputs,time.monotonic_ns())
    assert time.monotonic()-start < 3
    assert worker.process.poll() is not None
    worker.close()


def test_invalid_input_closes_worker(inputs):
    worker=GeometryWorker()
    bad=(inputs[0][:1],*inputs[1:])
    with pytest.raises(RuntimeError,match='shapes disagree'):
        worker.compute(bad,time.monotonic_ns())
    assert worker.process.poll() is not None
    worker.close()


def test_oversized_message_rejected_before_allocation():
    import socket, struct
    a,b=socket.socketpair()
    try:
        a.sendall(struct.pack('!I',MAX_MESSAGE+1))
        with pytest.raises(ValueError,match='byte limit'):
            _recv(b,time.monotonic()+1)
    finally:
        a.close();b.close()


def test_mode_and_subscriber_lifecycle(monkeypatch,tmp_path):
    import oak_surroundings as module
    flag=tmp_path/'enabled'
    monkeypatch.setattr(module,'PROCESS_FLAG',flag)
    workers=[]
    class FakeWorker:
        def __init__(self):
            self.closed=False;self.process=NS(pid=123);workers.append(self)
        def close(self): self.closed=True
    monkeypatch.setattr(module,'GeometryWorker',FakeWorker)
    obj=module.OakSurroundings.__new__(module.OakSurroundings)
    obj.geometry_worker=None;obj.process_enabled=False;obj.worker_retry_at=0
    obj._sync_worker(True)
    assert not workers
    flag.touch();assert obj._sync_worker(True)
    assert len(workers)==1
    obj._sync_worker(True);assert len(workers)==1
    obj._sync_worker(False);assert workers[0].closed and obj.geometry_worker is None
    obj._sync_worker(True);assert len(workers)==2
    flag.unlink();obj._sync_worker(True)
    assert workers[1].closed and not obj.process_enabled


def test_high_detail_budget_crosses_old_cap():
    inputs=(np.full((512,384,3),128,np.uint8),np.full((512,384),1000,np.uint16),
            np.ones((512,384),bool),np.array([[541,0,192],[0,541,256],[0,0,1]]),.0821,12000)
    worker=GeometryWorker()
    try:
        actual=worker.compute(inputs,time.monotonic_ns())
        assert 6000<len(actual[0])<=12000
        for a,b in zip(actual,refine(*inputs)):np.testing.assert_array_equal(a,b)
    finally:worker.close()
