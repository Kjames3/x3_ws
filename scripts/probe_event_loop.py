#!/usr/bin/env python3
"""Measure x3_server broadcast_loop period distribution + OAK depth fps.
Runs on the robot over localhost so network jitter can't contaminate the signal.
"""
import asyncio, json, sys, time, statistics, collections
import websockets

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
URL = "ws://127.0.0.1:8081"

async def main():
    stamps, fps_depth, ndet, types = [], [], [], collections.Counter()
    nbin = 0
    async with websockets.connect(URL, max_size=None, ping_interval=None) as ws:
        t0 = time.monotonic()
        while time.monotonic() - t0 < DUR:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5.0)
            except asyncio.TimeoutError:
                break
            if isinstance(msg, (bytes, bytearray)):
                nbin += 1
                continue
            try:
                d = json.loads(msg)
            except Exception:
                continue
            t = d.get("type", "?")
            types[t] += 1
            # the telemetry/sensor broadcast carries the fps block
            if t == "readout":
                stamps.append(time.monotonic())
                det = d.get("oak_detections")
                if isinstance(det, list):
                    ndet.append(len(det))
            # fps block rides on the slow readout
            f = d.get("fps_oak_depth")
            if f is None and isinstance(d.get("fps"), dict):
                f = d["fps"].get("fps_oak_depth") or d["fps"].get("oak_depth")
            if isinstance(f, (int, float)):
                fps_depth.append(f)

    if len(stamps) < 10:
        print(json.dumps({"error": "insufficient telemetry",
                          "types": dict(types), "binary": nbin,
                          "n": len(stamps)}))
        return
    per = [ (stamps[i+1]-stamps[i])*1000.0 for i in range(len(stamps)-1) ]
    per_s = sorted(per)
    def pct(p):
        return per_s[min(len(per_s)-1, int(len(per_s)*p))]
    out = {
        "n_samples": len(per),
        "duration_s": round(stamps[-1]-stamps[0], 2),
        "rate_hz": round(len(per)/(stamps[-1]-stamps[0]), 3),
        "target_hz": 20.0,
        "period_ms": {
            "mean": round(statistics.fmean(per), 2),
            "p50": round(pct(0.50), 2),
            "p90": round(pct(0.90), 2),
            "p95": round(pct(0.95), 2),
            "p99": round(pct(0.99), 2),
            "max": round(per_s[-1], 2),
            "stdev": round(statistics.pstdev(per), 2),
        },
        "stall_gt_100ms": sum(1 for x in per if x > 100),
        "stall_gt_200ms": sum(1 for x in per if x > 200),
        "fps_oak_depth_mean": round(statistics.fmean(fps_depth), 2) if fps_depth else None,
        "detections_mean": round(statistics.fmean(ndet), 3) if ndet else None,
        "binary_frames": nbin,
        "types": dict(types),
    }
    print(json.dumps(out, indent=2))

asyncio.run(main())
