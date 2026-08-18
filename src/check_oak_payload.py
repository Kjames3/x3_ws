"""Payload-level parity check for the OAK-D data path.

Confirms the server is actually delivering OAK content to clients -- not just that
messages arrive at the right rate. Run on the robot: python3 check_oak_payload.py
"""
import asyncio
import json
import sys
import time

import websockets

URL = "ws://127.0.0.1:8081"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


async def main():
    imu_ok = depth_fps = 0
    imu_samples, fps_samples, det_counts = [], [], []
    n_readout = 0
    async with websockets.connect(URL, max_size=None, ping_interval=None) as ws:
        t0 = time.monotonic()
        while time.monotonic() - t0 < DUR:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5.0)
            except asyncio.TimeoutError:
                break
            if isinstance(msg, (bytes, bytearray)):
                continue
            try:
                d = json.loads(msg)
            except Exception:
                continue
            if d.get("type") == "readout":
                n_readout += 1
                imu = d.get("oak_imu")
                if isinstance(imu, dict) and imu:
                    imu_samples.append(imu)
                det = d.get("oak_detections")
                if isinstance(det, list):
                    det_counts.append(len(det))
            f = d.get("fps_oak_depth")
            if isinstance(f, (int, float)):
                fps_samples.append(f)

    print(f"readouts:            {n_readout}")
    print(f"oak_imu populated:   {len(imu_samples)}/{n_readout}")
    if imu_samples:
        k = sorted(imu_samples[-1].keys())
        print(f"  imu keys:          {k}")
        print(f"  last sample:       {imu_samples[-1]}")
    print(f"fps_oak_depth samples: {len(fps_samples)} "
          f"last={fps_samples[-1] if fps_samples else None}")
    print(f"detection frames:    {len(det_counts)} "
          f"max_in_frame={max(det_counts) if det_counts else None}")

    ok = (n_readout > 0 and len(imu_samples) > 0
          and fps_samples and fps_samples[-1] > 5.0)
    print("PARITY CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


sys.exit(asyncio.run(main()))
