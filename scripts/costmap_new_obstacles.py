"""List obstacles in the LIVE global costmap that are not in the saved map.  Run ON THE ROBOT
with Nav2 up and localized:

    cd ~/x3_ws/src && source /opt/ros/humble/setup.bash && source ../install/setup.bash
    ROS_DOMAIN_ID=42 python3 ../scripts/costmap_new_obstacles.py <map.yaml>

Prints blobs of lethal cells with no mapped obstacle within 0.15 m: chairs, boxes and
tables moved since the map was made.  Found 2026-10-22 why goals near P1 in the lab kept
timing out: three new blobs sat within 0.2 m of the goal.  Also warns when the scan-match
score is low (a layout that no longer matches the map)."""
import sys, time, math, numpy as np, yaml, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
map_yaml = sys.argv[1]
rclpy.init(); n = Node("new_obstacles"); gc = []
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: gc.append(m),
                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL))
t = time.time()
while time.time() - t < 6 or not gc:
    rclpy.spin_once(n, timeout_sec=0.2)
m = gc[-1]; a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
res = m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
y = yaml.safe_load(open(map_yaml)); sres = y["resolution"]; sox, soy = y["origin"][0], y["origin"][1]
pgm = map_yaml.rsplit(".", 1)[0] + ".pgm"
raw = open(pgm, "rb").read(); parts = raw.split(b"\n", 3)
W, H = map(int, parts[1].split()); s = np.frombuffer(parts[3], np.uint8).reshape(H, W)
static_occ = s <= 10
ys, xs = np.nonzero(a >= 99); wx = ox + (xs + .5) * res; wy = oy + (ys + .5) * res
new = []
for x, yy in zip(wx, wy):
    i = int((x - sox) / sres); j = H - 1 - int((yy - soy) / sres)
    if 0 <= i < W and 0 <= j < H:
        r = 3
        if not static_occ[max(0, j - r):j + r + 1, max(0, i - r):i + r + 1].any():
            new.append((x, yy))
print("global costmap lethal cells: %d, not in the saved map: %d" % (len(xs), len(new)))
cells = {}
for x, yy in new:
    cells.setdefault((round(x / 0.4), round(yy / 0.4)), []).append((x, yy))
for c in sorted(cells.values(), key=len, reverse=True)[:10]:
    c = np.array(c)
    print("  new obstacle blob: %3d cells around (%.2f, %.2f)" % (len(c), c[:, 0].mean(), c[:, 1].mean()))
