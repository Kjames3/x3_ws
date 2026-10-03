import sys, math
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from foot_diagnostic import select_ankle_component

# Frozen pre-optimization oracle: preserve the exact selected support pixels.
def reference_component(z, valid, ankle, radius):
    """Choose the nearest supported depth layer, not a crop-wide percentile.

    Probe multiple layers; reject speckles and components remote from the ankle.
    A foreground shoe can occupy less than 20% of the crop while the shin
    dominates it. A single percentile then silently changes physical surfaces.
    """
    seeds = np.unique(np.percentile(z[valid], [1, 3, 5, 10, 20, 40, 60, 80]))
    minimum = max(16, int(math.ceil(.01*valid.sum())))
    best = None
    best_depth = float('inf')
    best_distance = float('inf')
    for seed in seeds:
        mask = (valid & (np.abs(z-seed) <= .10)).astype(np.uint8)
        n, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        for j in range(1, n):
            if (stats[j, cv2.CC_STAT_AREA] < minimum
                    or np.linalg.norm(centers[j]-ankle) > radius):
                continue
            use = labels == j
            depth = float(np.median(z[use]))
            distance = float(np.linalg.norm(centers[j]-ankle))
            if depth < best_depth-.02 or (abs(depth-best_depth) <= .02 and distance < best_distance):
                best, best_depth, best_distance = use, depth, distance
    return best



def test_optimized_support_matches_original():
    rng=np.random.default_rng(818)
    for i in range(250):
        z=rng.choice([.5,.52,.6,.8,1.,1.3],size=(41,41)).astype('float32')
        if i%2:
            z[:]=1.3
            z[12:29,15:30]=rng.choice([.8,.81,.82],size=(17,15))
        valid=rng.random(z.shape)>.1
        expected=reference_component(z,valid,(20,20),20)
        actual=select_ankle_component(z,valid,(20,20),20)
        assert (expected is None)==(actual is None)
        if expected is not None:np.testing.assert_array_equal(actual,expected)
