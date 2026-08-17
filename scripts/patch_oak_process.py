#!/usr/bin/env python3
"""Patch server_x3.py to support --oak-process (out-of-process OAK-D driver).

Idempotent and reversible: writes server_x3.py.oakproc.bak on first run and
refuses to proceed if any anchor is missing or ambiguous. Run with --revert to
restore the backup.

The robot's server_x3.py diverges from the repo copy, so this edits in place
rather than shipping a whole file.
"""
import argparse
import os
import shutil
import sys

TARGET = "/home/jetson/x3_ws/src/server_x3.py"
BACKUP = TARGET + ".oakproc.bak"

ARG_ANCHOR = "args = parser.parse_args()\n"
ARG_INSERT = (
    "parser.add_argument('--oak-process', action='store_true', dest='oak_process',\n"
    "                    help='Run the OAK-D driver in a separate process (shared-memory\\n'\n"
    "                         'transport) so its capture and host-side decode never hold\\n'\n"
    "                         'this process GIL and stall the asyncio event loop.')\n"
)

RATE_ANCHOR = "OAK_ROS_RATE = args.oak_ros_rate\n"
RATE_INSERT = "OAK_PROCESS = args.oak_process\n"

IMPORT_ANCHOR = "            from oakd_driver import OakDCamera\n"
IMPORT_INSERT = (
    "            if OAK_PROCESS:\n"
    "                from oakd_process import OakDCameraProcess as OakDCamera\n"
    "            else:\n"
    "                from oakd_driver import OakDCamera\n"
)

MARKER = "--oak-process"


def apply():
    src = open(TARGET).read()
    if MARKER in src:
        print("already patched; nothing to do")
        return 0

    for name, anchor in (("parse_args", ARG_ANCHOR),
                         ("OAK_ROS_RATE", RATE_ANCHOR),
                         ("oakd_driver import", IMPORT_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"ABORT: anchor {name!r} found {n} times (expected exactly 1)")
            return 1

    if not os.path.exists(BACKUP):
        shutil.copy2(TARGET, BACKUP)
        print(f"backup written: {BACKUP}")

    out = src.replace(ARG_ANCHOR, ARG_INSERT + ARG_ANCHOR)
    out = out.replace(RATE_ANCHOR, RATE_ANCHOR + RATE_INSERT)
    out = out.replace(IMPORT_ANCHOR, IMPORT_INSERT)

    tmp = TARGET + ".tmp"
    with open(tmp, "w") as f:
        f.write(out)
    import py_compile
    try:
        py_compile.compile(tmp, doraise=True)
    except py_compile.PyCompileError as e:
        os.unlink(tmp)
        print(f"ABORT: patched file does not compile: {e}")
        return 1
    os.replace(tmp, TARGET)
    print("patched OK (+%d lines)" % (out.count("\n") - src.count("\n")))
    return 0


def revert():
    if not os.path.exists(BACKUP):
        print("no backup found; nothing to revert")
        return 1
    shutil.copy2(BACKUP, TARGET)
    print(f"reverted {TARGET} from {BACKUP}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    sys.exit(revert() if a.revert else apply())
