import json
import numpy as np
from oak_display_diagnostic import DisplayDiagnostic


def test_capture_is_bounded_and_owns_arrays(tmp_path, monkeypatch):
    import oak_display_diagnostic as module
    jobs = []
    # Defer writer to prove saved arrays aren't aliases of camera buffers.
    monkeypatch.setattr(module.threading, 'Thread', lambda target,args,**kw:
                        type('Job', (), {'start':lambda self:jobs.append((target,args))})())
    d = DisplayDiagnostic(tmp_path/'capture')
    assert not any(d.wants_sample() for _ in range(d.WARMUP))
    assert d.wants_sample()
    inputs = [np.ones((2,2)) for _ in range(5)]
    outputs = [np.ones((2,2)) for _ in range(4)]
    for i in range(20):
        d.capture(inputs, outputs, {}, ['test',0,i])
    assert d.done and not d.frames and len(jobs)==1 and not d.wants_sample()
    inputs[0][:] = 99
    fn,args = jobs[0]; fn(*args)
    manifest = json.loads((d.output/'manifest.json').read_text())
    assert len(manifest['frames'])==10
    with np.load(d.output/'frame-00.npz') as f:
        assert (f['rgb']==1).all()
