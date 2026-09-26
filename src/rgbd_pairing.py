"""Bounded one-to-one timestamp pairing for the existing OAK camera owner."""
from collections import deque, Counter


def timedelta_ns(value):
    # Avoid floating-point total_seconds precision loss at long host uptimes.
    return ((value.days * 86400 + value.seconds) * 1000000 + value.microseconds) * 1000


class RGBDPairer:
    def __init__(self, max_skew_ns=20_000_000, capacity=16):
        self.max_skew_ns = max_skew_ns
        self.capacity = capacity
        self.depth = deque()
        self.rgb = deque()
        self.last = {}
        self.counters = Counter()
        self.epoch = 0
        self.pair_seq = 0

    def add(self, stream, packet):
        meta = packet['meta']
        identity = (meta['seq'], meta['device_ns'])
        previous = self.last.get(stream)
        if previous is not None:
            if identity == previous:
                self.counters[stream + '_duplicate'] += 1
                return []
            if identity[0] <= previous[0] or identity[1] < previous[1]:
                self.counters['reset_or_reorder'] += 1
                self.counters['depth_unpaired'] += len(self.depth)
                self.counters['rgb_unpaired'] += len(self.rgb)
                self.depth.clear(); self.rgb.clear(); self.last.clear()
                self.epoch += 1
            elif identity[0] > previous[0] + 1:
                self.counters[stream + '_source_gaps'] += identity[0] - previous[0] - 1
        self.last[stream] = identity
        queue = self.depth if stream == 'depth' else self.rgb
        queue.append(packet)
        if len(queue) > self.capacity:
            queue.popleft()
            self.counters[stream + '_overflow'] += 1
        return self._pair()

    def _pair(self):
        pairs = []
        while self.rgb and self.depth:
            rgb = self.rgb[0]
            t = rgb['meta']['device_ns']
            # Wait for a depth at/after RGB to choose the nearest bracketing sample.
            if self.depth[-1]['meta']['device_ns'] < t:
                break
            index = min(range(len(self.depth)), key=lambda i: abs(self.depth[i]['meta']['device_ns'] - t))
            depth = self.depth[index]
            skew = depth['meta']['device_ns'] - t
            self.rgb.popleft()
            if abs(skew) > self.max_skew_ns:
                self.counters['rgb_unpaired'] += 1
                continue
            for _ in range(index):
                self.depth.popleft(); self.counters['depth_unpaired'] += 1
            self.depth.popleft()
            self.pair_seq += 1
            self.counters['paired'] += 1
            pairs.append(dict(rgb=rgb, depth=depth, pair_seq=self.pair_seq,
                              epoch=self.epoch, skew_ns=skew, counters=dict(self.counters)))
        return pairs
