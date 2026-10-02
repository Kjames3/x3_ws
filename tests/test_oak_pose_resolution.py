"""Exercise actual config loading and decoder without opening OAK hardware."""
import ast
import json
import logging
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def decoder_class():
    tree = ast.parse((ROOT/'src/oakd_driver.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OakDCamera')
    wanted = {'_load_nn_config', '_nn_rows', '_decode'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in wanted]
    scope = dict(np=np, json=json, logger=logging.getLogger(__name__),
                 _sigmoid=lambda x: 1/(1+np.exp(-x)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 'oak-decoder', 'exec'), scope)
    return scope['OakDCamera']


@pytest.mark.parametrize('model,h,w,n', [('yolo26n-pose',640,480,6300),
                                       ('yolo26n-pose-512',512,384,4032)])
def test_configured_pose_output_is_decodable(model,h,w,n):
    camera = decoder_class()()
    camera.nn_conf, camera.nn_iou = .5,.5
    camera.labels = ['person']
    camera._N_ANCHORS = 6300
    camera._load_nn_config(ROOT/'src/blobs'/model/'config.json')
    assert (camera.nn_h,camera.nn_w,camera._N_ANCHORS)==(h,w,n)
    raw=np.zeros((56,n),dtype=np.float32)
    raw[:5,17]=[w/2,h/2,80,160,.95]
    raw[5:,17]=np.tile([w/2,h*.8,.9],17)
    # This is the production early-return guard that discarded smaller tensors.
    assert raw.size >= camera._nn_rows*camera._N_ANCHORS
    boxes,scores,classes,keypoints=camera._decode(raw.ravel(),('cm',False))
    assert scores.argmax()==17 and scores[17]>.9
    assert boxes.shape==(n,4) and keypoints.shape==(n,17,3)
    np.testing.assert_allclose(keypoints[17,15],[w/2,h*.8,.9],rtol=1e-6)
