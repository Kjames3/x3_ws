"""One-shot bounded display diagnostic; no camera ownership or runtime tuning."""
import json
import logging
from pathlib import Path
import threading
import time

import cv2
import numpy as np

log = logging.getLogger(__name__)


class DisplayDiagnostic:
    LIMIT = 10
    WARMUP = 15

    def __init__(self, output):
        self.output = Path(output)
        self.seen = 0
        self.frames = []
        self.done = False

    @classmethod
    def arm(cls):
        trigger = Path('/tmp/x3_oak_profile_once')
        try:
            trigger.unlink()
        except FileNotFoundError:
            return None
        root = Path(__file__).resolve().parents[1]
        return cls(root / 'artifacts' / ('oak-matched-profile-' + time.strftime('%Y%m%d-%H%M%S')))

    def wants_sample(self):
        self.seen += 1
        return not self.done and self.seen > self.WARMUP

    def capture(self, inputs, outputs, timings, identity):
        if self.done:
            return
        # Copy after cloud publication and outside geometry timings. Never retain
        # mutable camera buffers. Ten frames bound retained memory/disk use.
        arrays = {name: np.array(value, copy=True) for name, value in zip(
            ('rgb', 'depth', 'support', 'k', 'correction', 'xyz', 'colors', 'confidence', 'pixels'),
            (*inputs[:5], *outputs))}
        arrays["budget"] = np.array(inputs[5] if len(inputs)>5 else 6000)
        self.frames.append((arrays, {'identity': identity, 'timings': timings}))
        if len(self.frames) == self.LIMIT:
            self.done = True
            frames, self.frames = self.frames, []
            threading.Thread(target=self._write, args=(frames,),
                             name='oak-profile-write', daemon=True).start()

    def _write(self, frames):
        try:
            self.output.mkdir(parents=True, exist_ok=False)
            records = []
            for i, (arrays, metadata) in enumerate(frames):
                name = f'frame-{i:02d}.npz'
                np.savez(self.output / name, **arrays)
                records.append(dict(metadata, file=name))
            # Manifest last: replay only considers a completed capture.
            (self.output / 'manifest.json').write_text(json.dumps({
                'frames': records, 'opencv_threads': cv2.getNumThreads(),
                'note': 'Copies after publish; file writes after all geometry samples. Diagnostic perturbs scheduling.'
            }, indent=2)+'\n')
            log.info('OAK matched profile saved: %s (%d frames)', self.output, len(frames))
        except Exception:
            log.exception('OAK matched profile write failed')
