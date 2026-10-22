"""Send one Nav2 goal through the server (the same message the GUI sends) and watch it.

    python3 scripts/send_nav_goal.py 1.57 -2.14        # drive to map (x, y)
    python3 scripts/send_nav_goal.py --cancel           # stop: cancel the active goal

Prints the navigation state about every 2 s, exits when the goal ends, and cancels the goal
on Ctrl-C or after --timeout seconds (default 90).  Host defaults to x3 (Tailscale name).
"""
import argparse, asyncio, json, sys, time
import websockets

async def run(args):
    async with websockets.connect(f"ws://{args.host}:8081", max_size=None, ping_interval=None) as ws:
        if args.cancel:
            await ws.send(json.dumps({"type": "cancel_nav"}))
            await asyncio.sleep(1.0)
            print("cancel sent")
            return
        await ws.send(json.dumps({"type": "set_nav_goal", "x": args.x, "y": args.y, "theta": 0.0}))
        t0 = last = time.time()
        seen_executing = False
        print(f"goal sent: ({args.x:.2f}, {args.y:.2f})  [Ctrl-C cancels]", flush=True)
        try:
            while time.time() - t0 < args.timeout:
                try:
                    m = await asyncio.wait_for(ws.recv(), 3)
                except asyncio.TimeoutError:
                    continue
                if not isinstance(m, str):
                    continue
                d = json.loads(m)
                if d.get("type") != "readout_slow":
                    continue
                nav = d.get("nav", {})
                st = nav.get("state")
                if time.time() - last >= 2.0:
                    last = time.time()
                    dist = nav.get("dist")
                    print(f"  t={time.time()-t0:4.0f}s  {st}  remaining {dist if dist is None else round(dist, 2)}", flush=True)
                if st == "EXECUTING":
                    seen_executing = True
                if st in ("SUCCEEDED", "FAILED", "ABORTED", "CANCELLED") and time.time() - t0 > 2:
                    print(f"goal ended: {st} after {time.time()-t0:.1f} s")
                    return
                if st == "IDLE" and seen_executing:
                    # the server reports IDLE (not CANCELLED) once a goal was cancelled or dropped
                    print(f"goal ended: cancelled or stopped (state IDLE) after {time.time()-t0:.1f} s")
                    return
            print("timeout: cancelling")
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("interrupted: cancelling")
        await ws.send(json.dumps({"type": "cancel_nav"}))
        await asyncio.sleep(1.0)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float, nargs="?")
    ap.add_argument("y", type=float, nargs="?")
    ap.add_argument("--cancel", action="store_true")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--host", default="x3")
    a = ap.parse_args()
    if not a.cancel and (a.x is None or a.y is None):
        ap.error("give x and y, or --cancel")
    try:
        asyncio.run(run(a))
    except KeyboardInterrupt:
        pass
