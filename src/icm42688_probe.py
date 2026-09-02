#!/usr/bin/env python3
"""Read-only bring-up probe for the ICM-42688-P on SPI1 (/dev/spidev0.0).

Wiring (Orin Nano 40-pin):  VCC=1 (3.3V)  GND=6  MOSI=19  MISO=21  SCK=23  CS=24

  --probe   WHO_AM_I only. Writes NOTHING. Use this first to prove the wiring.
  --stream  Enables accel+gyro (low-noise) and prints scaled samples.
  --i2c     Talk over I2C (default bus 7 = pins 3/5) instead of SPI.

I2C wiring: VCC=1(3.3V) GND=6 SDA=3 SCL=5, CS tied HIGH to 3.3V to force I2C
mode (the part only enters SPI on a CS falling edge). AD0 low => 0x68.

Mounting on this robot: sensor Z=forward, X=left, Y=up.  Sensor->ROS(REP-103)
is a pure cyclic permutation (det +1), so there are NO sign flips:
    ros_x(fwd) = sensor_z    ros_y(left) = sensor_x    ros_z(up) = sensor_y
At rest the raw accel should therefore read y ~ +9.81, x ~ 0, z ~ 0.
"""
import argparse
import sys
import time

WHO_AM_I      = 0x75
EXPECTED_WHOAMI = 0x47
REG_BANK_SEL  = 0x76
PWR_MGMT0     = 0x4E
GYRO_CONFIG0  = 0x4F
ACCEL_CONFIG0 = 0x50
ACCEL_DATA_X1 = 0x1F          # 12 bytes: accel xyz then gyro xyz, all big-endian

GYRO_SENS_LSB_PER_DPS = 16.4     # +/-2000 dps  (FS_SEL=0)
ACCEL_SENS_LSB_PER_G  = 2048.0   # +/-16 g      (FS_SEL=0)
DEG2RAD = 3.141592653589793 / 180.0
G       = 9.80665


class ICM42688I2C:
    """Same register interface over I2C, so the rest of the script is unchanged."""

    def __init__(self, bus=7, addr=0x68):
        import smbus2
        self.bus = smbus2.SMBus(bus)
        self.addr = addr

    def read(self, reg, n=1):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    who_am_i = lambda self: self.read(WHO_AM_I)[0]

    def close(self):
        self.bus.close()


class ICM42688:
    def __init__(self, bus=0, dev=0, hz=8_000_000):
        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = hz
        self.spi.mode = 0b00           # CPOL=0 CPHA=0; the part also accepts mode 3

    def read(self, reg, n=1):
        out = self.spi.xfer2([reg | 0x80] + [0x00] * n)
        return out[1:]

    def write(self, reg, val):
        self.spi.xfer2([reg & 0x7F, val])

    def who_am_i(self):
        return self.read(WHO_AM_I)[0]

    def enable(self, odr=0x07):        # 0x07 = 200 Hz, 0x06 = 1 kHz, 0x08 = 100 Hz
        self.write(REG_BANK_SEL, 0x00)
        self.write(GYRO_CONFIG0, odr)          # FS_SEL=0 -> +/-2000 dps
        self.write(ACCEL_CONFIG0, odr)         # FS_SEL=0 -> +/-16 g
        self.write(PWR_MGMT0, 0x0F)            # gyro + accel, low-noise mode
        time.sleep(0.05)                       # gyro needs ~30 ms to start

    def sample(self):
        b = self.read(ACCEL_DATA_X1, 12)
        def s16(hi, lo):
            v = (b[hi] << 8) | b[lo]
            return v - 65536 if v & 0x8000 else v
        ax, ay, az = s16(0, 1), s16(2, 3), s16(4, 5)
        gx, gy, gz = s16(6, 7), s16(8, 9), s16(10, 11)
        return (
            ax / ACCEL_SENS_LSB_PER_G * G,
            ay / ACCEL_SENS_LSB_PER_G * G,
            az / ACCEL_SENS_LSB_PER_G * G,
            gx / GYRO_SENS_LSB_PER_DPS * DEG2RAD,
            gy / GYRO_SENS_LSB_PER_DPS * DEG2RAD,
            gz / GYRO_SENS_LSB_PER_DPS * DEG2RAD,
        )

    def close(self):
        self.spi.close()


ICM42688I2C.enable = ICM42688.enable
ICM42688I2C.sample = ICM42688.sample


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--probe', action='store_true', help='read WHO_AM_I only, no writes')
    ap.add_argument('--stream', action='store_true', help='enable sensors and print samples')
    ap.add_argument('--i2c', action='store_true', help='use I2C instead of SPI')
    ap.add_argument('--bus', type=int, default=None, help='SPI bus (def 0) or I2C bus (def 7)')
    ap.add_argument('--dev', type=int, default=0)
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x68)
    ap.add_argument('--seconds', type=float, default=5.0)
    args = ap.parse_args()
    if not (args.probe or args.stream):
        args.probe = True

    if args.i2c:
        bus = 7 if args.bus is None else args.bus
        try:
            imu = ICM42688I2C(bus, args.addr)
            imu.who_am_i()
        except FileNotFoundError:
            sys.exit(f'/dev/i2c-{bus} not present')
        except OSError as e:
            sys.exit(f'no ACK from 0x{args.addr:02X} on i2c-{bus} ({e}).\n'
                     f'  Try `i2cdetect -y -r {bus}`. If 0x68 and 0x69 are both absent the\n'
                     f'  part is not powered or SDA/SCL are not on pins 3/5. If it shows at\n'
                     f'  0x69 instead, AD0 is high -- rerun with --addr 0x69.')
        print(f'I2C bus {bus}, address 0x{args.addr:02X}')
    else:
        bus = 0 if args.bus is None else args.bus
        try:
            imu = ICM42688(bus, args.dev)
        except FileNotFoundError:
            sys.exit(f'/dev/spidev{bus}.{args.dev} not present -- SPI1 is not enabled')
        except PermissionError:
            sys.exit(f'permission denied on /dev/spidev{bus}.{args.dev} -- udev rule or sudo')

    wai = imu.who_am_i()
    print(f'WHO_AM_I = 0x{wai:02X} (expect 0x{EXPECTED_WHOAMI:02X})')
    if wai != EXPECTED_WHOAMI:
        if wai in (0x00, 0xFF):
            print('  0x00/0xFF means no reply at all. Most likely MOSI/MISO are swapped')
            print('  (pin 19 must go to the breakout data-IN, pin 21 to data-OUT), or CS')
            print('  is not on pin 24, or the part has no 3.3 V.')
        else:
            print('  Responding but not an ICM-42688-P -- check CS is not shared.')
        imu.close()
        sys.exit(1)
    print('  OK: ICM-42688-P identified.')

    if not args.stream:
        imu.close()
        return

    imu.enable()
    print('\n  raw sensor frame (m/s^2, rad/s)          remapped to ROS body frame')
    t_end = time.time() + args.seconds
    n = 0
    while time.time() < t_end:
        ax, ay, az, gx, gy, gz = imu.sample()
        # sensor -> ROS: x=sz, y=sx, z=sy  (no sign flips)
        print(f'  a=({ax:+6.2f},{ay:+6.2f},{az:+6.2f}) g=({gx:+6.3f},{gy:+6.3f},{gz:+6.3f})'
              f'   a=({az:+6.2f},{ax:+6.2f},{ay:+6.2f}) g=({gz:+6.3f},{gx:+6.3f},{gy:+6.3f})')
        n += 1
        time.sleep(0.1)
    imu.close()
    print(f'\n{n} samples. At rest expect RAW ay ~ +9.81 -> ROS az ~ +9.81.')


if __name__ == '__main__':
    main()
