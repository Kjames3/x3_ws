import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import smbus2
def probe():
    for bus_num in [0, 1, 7, 8]:
        try:
            bus = smbus2.SMBus(bus_num)
        except Exception:
            continue
        print(f"Scanning Bus {bus_num}...")
        for addr in range(0x40, 0x50):
            try:
                bus.read_byte(addr)
                print(f"  Found device at 0x{addr:02X} on bus {bus_num}")
            except Exception:
                pass
probe()
