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
