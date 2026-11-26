"""Run the production cleaner without importing the hardware server."""
import ast
import base64
import math
from pathlib import Path


def test_cleaner():
    path = Path(__file__).resolve().parents[2] / 'src/server_x3.py'
    tree = ast.parse(path.read_text())
    tree.body = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == '_clean_landmarks'
        or isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == '_LANDMARK_NUM' for t in node.targets)
    ]
    scope = dict(math=math, base64=base64)
    exec(compile(tree, str(path), 'exec'), scope)
    clean = scope['_clean_landmarks']
    obj = dict(kind='chair', X=1, Y=2, yaw=0, L=.5, D=.5, H=.9)
    record = bytes([128, 128, 20, 50, 80, 200, 3])
    encode = lambda raw: base64.b64encode(raw).decode('ascii')
    valid = dict(obj, lookalike=encode(b'\x01\x09' + record * 4000))
    assert clean([valid]) == [valid]
    for value in ('bad base64', [], valid['lookalike'] * 2,
                  encode(b'\x02\x09' + record), encode(b'\x01\x09' + record[:-1]),
                  encode(b'\x01\x09' + record[:-1] + b'\x01')):
        assert clean([dict(obj, lookalike=value)]) == [obj]
    assert clean([obj]) == [obj]


if __name__ == '__main__':
    test_cleaner()
    print('Landmark cleaner accepted legacy and full-cap models, stripped malformed data')
