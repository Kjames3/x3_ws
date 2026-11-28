#!/usr/bin/env python3
"""Room-inventory capture, camera half: save every camera frame the x3 GUI websocket
streams (front = OAK left mono 640x400, rear, left, right) plus readout telemetry
(robot_pose, oak_detections) for N seconds. Run ON THE ROBOT next to a raw Livox bag,
robot parked:

    D=~/bags/room_inventory_$(date +%F)/cap_$(date +%H%M); mkdir -p $D; cd $D
    source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash   # CustomMsg type
    source ~/x3_ws/install/setup.bash   # + ROS_DOMAIN_ID=42 and discovery-server env
    timeout -s INT 14 ros2 bag record -o lidar /livox/lidar /livox/imu /tf /tf_static /odom \\
        /surroundings/points /surroundings/pose &
    sleep 2; python3 ~/x3_ws/src/tools/perception/room_capture.py . 10; wait

Frames land in <out>/<role>/<wall time>.jpg (same clock as the bag on the robot).
First capture: ~/bags/room_inventory_2026-11-28/cap_0328 (apartment, 10 s).
"""
import asyncio, json, os, sys, time, websockets
ROLES = ("front", "rear", "left", "right")
async def main(out, dur):
    for r in ROLES: os.makedirs(f"{out}/{r}", exist_ok=True)
    ws = await websockets.connect("ws://127.0.0.1:8081", max_size=None)
    t0 = time.time(); n = {}
    with open(f"{out}/telemetry.ndjson", "w") as tf:
        while time.time() - t0 < dur:
            m = await ws.recv(); t = time.time()
            if isinstance(m, bytes):
                if m[:4] == b"CAM1":
                    role = ROLES[m[4]]; i = m.find(b"\xff\xd8")
                    open(f"{out}/{role}/{t:.3f}.jpg", "wb").write(m[i:]); n[role] = n.get(role, 0) + 1
            else:
                j = json.loads(m)
                if j.get("type") == "readout":
                    tf.write(json.dumps({"t": t, "robot_pose": j.get("robot_pose"), "oak_detections": j.get("oak_detections")}) + "\n")
    print("frames", n)
asyncio.run(main(sys.argv[1], float(sys.argv[2])))
