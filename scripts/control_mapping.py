#!/usr/bin/env python3
"""
control_mapping.py
------------------
Helper utility to interact with the Yahboom X3 WebSocket server (server_x3.py)
running on the Jetson Orin Nano.

This allows launching/stopping mapping (SLAM + Nav2) and saving maps directly
from the command line.

Usage:
  python3 scripts/control_mapping.py <JETSON_IP> start
  python3 scripts/control_mapping.py <JETSON_IP> stop
  python3 scripts/control_mapping.py <JETSON_IP> status
  python3 scripts/control_mapping.py <JETSON_IP> save <map_name>
"""

import sys
import json
import asyncio
import argparse
import websockets

# ANSI Color Codes for premium UI look
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

async def receive_until_type(ws, target_type, timeout=5.0):
    """Receive WebSocket messages until a specific type is found, or timeout occurs."""
    start_time = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_time < timeout:
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=timeout)
            if isinstance(msg, bytes):
                # Ignore camera raw binary frames
                continue
            data = json.loads(msg)
            if data.get("type") == target_type:
                return data
        except json.JSONDecodeError:
            continue
        except asyncio.TimeoutError:
            break
    return None

async def print_status(jetson_ip, ws_port):
    uri = f"ws://{jetson_ip}:{ws_port}"
    print(f"{BLUE}[DDS Helper] Connecting to {uri}...{RESET}")
    try:
        async with websockets.connect(uri, timeout=3.0) as ws:
            # First message should be hello
            hello_msg = await receive_until_type(ws, "hello", timeout=2.0)
            if hello_msg:
                print(f"{GREEN}✓ Connected successfully in '{hello_msg.get('mode')}' mode!{RESET}")
            
            # Retrieve a readout message for status
            print(f"{BLUE}[DDS Helper] Fetching active system status...{RESET}")
            readout = await receive_until_type(ws, "readout", timeout=3.0)
            if not readout:
                print(f"{RED}✗ Failed to fetch system readout from server.{RESET}")
                return 1

            nav_data = readout.get("nav", {})
            nav2_running = nav_data.get("nav2_running", False)
            slam_active = readout.get("slam_active", False)
            nav_phase = readout.get("nav_phase", "UNKNOWN")
            batt = readout.get("battery", {})
            volt = batt.get("voltage", 0.0)

            print("\n" + "="*50)
            print(f" {BOLD}{CYAN}YAHBOOM X3 — SYSTEM STATUS{RESET} ")
            print("="*50)
            print(f"  {BOLD}Jetson IP:{RESET}      {jetson_ip}")
            print(f"  {BOLD}Battery:{RESET}        {volt:.2f} V")
            print(f"  {BOLD}SLAM Active:{RESET}    " + (f"{GREEN}RUNNING{RESET}" if slam_active else f"{RED}STOPPED{RESET}"))
            print(f"  {BOLD}Nav2 Stack:{RESET}     " + (f"{GREEN}RUNNING{RESET}" if nav2_running else f"{RED}STOPPED{RESET}"))
            print(f"  {BOLD}Nav Phase:{RESET}      {YELLOW}{nav_phase}{RESET}")
            print("="*50 + "\n")
            return 0
    except Exception as exc:
        print(f"{RED}✗ Error: Could not connect to Jetson at {jetson_ip}:{ws_port}.{RESET}")
        print(f"  Make sure the x3_server service is active: {YELLOW}sudo systemctl status x3_server{RESET}")
        return 1

async def start_mapping(jetson_ip, ws_port):
    uri = f"ws://{jetson_ip}:{ws_port}"
    print(f"{BLUE}[DDS Helper] Connecting to {uri}...{RESET}")
    try:
        async with websockets.connect(uri, timeout=3.0) as ws:
            # Wait for hello
            await receive_until_type(ws, "hello", timeout=2.0)
            
            # Send command to launch Nav2 + SLAM Toolbox
            cmd = {"type": "launch_nav2", "slam": True}
            print(f"{BLUE}[DDS Helper] Sending command to start SLAM & Nav2 on Jetson...{RESET}")
            await ws.send(json.dumps(cmd))
            
            # Wait for launch result
            res = await receive_until_type(ws, "nav2_launch_result", timeout=8.0)
            if res and res.get("success"):
                print(f"{GREEN}✓ Nav2 + SLAM Toolbox successfully triggered!{RESET}")
                print(f"{GREEN}  Message: {res.get('msg')}{RESET}")
                print(f"{YELLOW}  Please wait a few seconds for the costmaps and map topics to initialize.{RESET}")
                return 0
            else:
                msg = res.get("msg") if res else "No response from server"
                print(f"{RED}✗ Failed to start mapping: {msg}{RESET}")
                return 1
    except Exception as exc:
        print(f"{RED}✗ Error connecting to websocket: {exc}{RESET}")
        return 1

async def stop_mapping(jetson_ip, ws_port):
    uri = f"ws://{jetson_ip}:{ws_port}"
    print(f"{BLUE}[DDS Helper] Connecting to {uri}...{RESET}")
    try:
        async with websockets.connect(uri, timeout=3.0) as ws:
            await receive_until_type(ws, "hello", timeout=2.0)
            
            print(f"{YELLOW}[DDS Helper] Stopping Nav2 & SLAM on Jetson...{RESET}")
            await ws.send(json.dumps({"type": "stop_nav2"}))
            await asyncio.sleep(0.5)
            await ws.send(json.dumps({"type": "stop_slam"}))
            print(f"{GREEN}✓ Sent shutdown commands to mapping stack.{RESET}")
            return 0
    except Exception as exc:
        print(f"{RED}✗ Error: {exc}{RESET}")
        return 1

async def save_map(jetson_ip, ws_port, map_name):
    uri = f"ws://{jetson_ip}:{ws_port}"
    print(f"{BLUE}[DDS Helper] Connecting to {uri}...{RESET}")
    try:
        async with websockets.connect(uri, timeout=3.0) as ws:
            await receive_until_type(ws, "hello", timeout=2.0)
            
            print(f"{BLUE}[DDS Helper] Saving map as '{map_name}'...{RESET}")
            await ws.send(json.dumps({"type": "save_map", "name": map_name}))
            
            # Wait for save results
            res = await receive_until_type(ws, "save_map_result", timeout=10.0)
            if res and res.get("success"):
                print(f"{GREEN}✓ Map successfully saved on Jetson!{RESET}")
                print(f"  Details: {res.get('message')}{RESET}")
                return 0
            else:
                msg = res.get("message") if res else "No response from server"
                print(f"{RED}✗ Failed to save map: {msg}{RESET}")
                return 1
    except Exception as exc:
        print(f"{RED}✗ Error: {exc}{RESET}")
        return 1

def main():
    parser = argparse.ArgumentParser(description="Yahboom X3 Mapping Remote Orchestrator")
    parser.add_argument("ip", help="Jetson Orin Nano IP Address")
    parser.add_argument("action", choices=["start", "stop", "status", "save"], help="Action to perform")
    parser.add_argument("map_name", nargs="?", default="classroom_map", help="Name to save map under (save action only)")
    parser.add_argument("--port", type=int, default=8081, help="WebSocket port (default: 8081)")
    args = parser.parse_args()

    if args.action == "status":
        ret = asyncio.run(print_status(args.ip, args.port))
    elif args.action == "start":
        ret = asyncio.run(start_mapping(args.ip, args.port))
    elif args.action == "stop":
        ret = asyncio.run(stop_mapping(args.ip, args.port))
    elif args.action == "save":
        ret = asyncio.run(save_map(args.ip, args.port, args.map_name))
    else:
        ret = 1

    sys.exit(ret)

if __name__ == "__main__":
    main()
