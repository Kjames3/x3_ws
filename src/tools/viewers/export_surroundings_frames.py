#!/usr/bin/env python3
"""Export a walk bag (+ GUI telemetry ndjson) to per-cloud frames for furniture_replay.cjs.

Usage (ROS sourced): export_surroundings_frames.py BAG.db3 TELEMETRY.ndjson OUT.ndjson
Each line: cloud xyz (base frame), its /surroundings/pose, the nearest earlier readout
robot_pose (cm), OAK detections and person positions (memory movers)."""
import sqlite3, json, math, sys, numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2 as pc2
db, tele, out = sys.argv[1:4]
con = sqlite3.connect(db)
tid = {n: i for i, n in con.execute("select id,name from topics")}
poses = []
for t, b in con.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid["/surroundings/pose"],)):
    m = deserialize_message(b, PoseStamped); q = m.pose.orientation
    poses.append(dict(stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, x=m.pose.position.x, y=m.pose.position.y,
                      yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
pstamps = np.array([p["stamp"] for p in poses])
# telemetry: robot_pose + oak detections by wall time
T = []
for l in open(tele):
    r = json.loads(l); m = r["msg"]
    if m.get("type") != "readout": continue
    T.append((r["t"], m.get("robot_pose"), m.get("oak_detections"), [dict(x=p.get("fwd"),y=p.get("left")) for p in (m.get("lidar_persons") or [])+(m.get("c3_tracks") or []) if p.get("fwd") is not None]))
tt = np.array([x[0] for x in T])
with open(out, "w") as f:
    for t, b in con.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid["/surroundings/points"],)):
        m = deserialize_message(b, PointCloud2)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.argmin(abs(pstamps - st)))
        if abs(pstamps[i] - st) > 1e-4: continue
        xyz = np.array([list(p) for p in pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)], dtype=np.float32).reshape(-1)
        j = int(np.searchsorted(tt, t / 1e9)) - 1
        if j < 0: continue
        tj, rp, dets, movers = T[j]
        f.write(json.dumps(dict(t=t/1e9, stamp=st, pose=poses[i], robot_pose=rp, dets=dets or [], det_t=tj, movers=movers,
                                xyz=[round(float(v), 3) for v in xyz])) + "\n")
