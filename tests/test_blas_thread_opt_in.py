import ast
import os
from pathlib import Path

import pytest


@pytest.mark.parametrize(('opt_in', 'existing_blas', 'existing_omp', 'expected'), [
    (None, None, None, (None, None)),
    ('1', None, None, ('1', '1')),
    ('2', '4', None, ('4', '2')),
    ('2', None, '3', ('2', '3')),
])
def test_blas_thread_environment(monkeypatch, opt_in, existing_blas, existing_omp, expected):
    server = Path(__file__).resolve().parents[1] / 'src' / 'server_x3.py'
    tree = ast.parse(server.read_text())
    # Run the startup block before any server dependencies are imported.
    assert isinstance(tree.body[1], ast.Import) and tree.body[1].names[0].name == 'os'
    assert isinstance(tree.body[2], ast.If)
    startup = ast.Module(body=tree.body[:3], type_ignores=[])
    monkeypatch.setattr(os, 'environ', os.environ.copy())
    for name, value in zip(('X3_BLAS_THREADS', 'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS'),
                           (opt_in, existing_blas, existing_omp)):
        if value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = value
    exec(compile(startup, str(server), 'exec'), {})
    assert (os.environ.get('OPENBLAS_NUM_THREADS'), os.environ.get('OMP_NUM_THREADS')) == expected
