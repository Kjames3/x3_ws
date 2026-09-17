"""PAA5100JE near-field optical flow sensor over Linux spidev.

The register init is PixArt's undocumented calibration sequence, copied verbatim
from Pimoroni's pmw3901-python (MIT License, Copyright (c) 2018 Pimoroni Ltd.,
https://github.com/pimoroni/pmw3901-python).  It is vendored rather than
imported because that package imports gpiod/gpiodevice at module load for an
optional GPIO chip-select this robot does not use, and neither is installed on
the Jetson.  Do not edit the table: the axis mapping and counts/m calibration
in flow_geometry were measured with exactly this init, and it does NOT write
REG_ORIENTATION (set_rotation was never called).

Wiring: /dev/spidev0.1 (pin 26 CS1), 3.3 V on pin 17, mode 0, 400 kHz.
"""
import fcntl
import os
import time

try:
    import spidev
except ImportError:          # laptop / CI: sim mode only
    spidev = None

WAIT = -1
REG_ID = 0x00
REG_DATA_READY = 0x02
REG_MOTION_BURST = 0x16
REG_POWER_UP_RESET = 0x3A

PRODUCT_ID = 0x49
BURST_LEN = 12


class PAA5100JE:
    def __init__(self, spi_bus=0, spi_cs=1, speed_hz=400000):
        if spidev is None:
            raise RuntimeError('spidev is not installed')
        # Exclusive advisory lock on the device node.  Every motion-burst read
        # CLEARS the chip's delta counters, so two readers split the counts
        # between them and each publishes roughly half the true velocity --
        # which happened (2026-09-16: a stale node plus a new one made a 3 m
        # drive integrate to ~2 m).  Fail loudly instead.
        self._lock_fd = os.open(f'/dev/spidev{spi_bus}.{spi_cs}', os.O_RDWR)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self._lock_fd)
            raise RuntimeError(
                f'/dev/spidev{spi_bus}.{spi_cs} is already in use by another PAA5100JE '
                'reader (another flow_node?); two readers split the motion counts') from None
        self.spi_dev = spidev.SpiDev()
        self.spi_dev.open(spi_bus, spi_cs)
        self.spi_dev.mode = 0
        self.spi_dev.max_speed_hz = speed_hz

        self._write(REG_POWER_UP_RESET, 0x5A)
        time.sleep(0.02)
        for offset in range(5):
            self._read(REG_DATA_READY + offset)
        self._secret_sauce()

        product_id, revision = self.get_id()
        if product_id != PRODUCT_ID or revision not in (0x00, 0x01):
            self.close()
            raise RuntimeError(
                f'not a PAA5100JE: id 0x{product_id:02x} rev 0x{revision:02x}')

    def get_id(self):
        return self._read(REG_ID, 2)

    def read_burst(self):
        """Raw 12-byte motion burst; parse with flow_geometry.parse_burst.

        Reading clears the chip's accumulated delta counters, so every call
        returns motion since the previous one.
        """
        return bytes(self.spi_dev.xfer2([REG_MOTION_BURST] + [0] * BURST_LEN)[1:])

    def close(self):
        try:
            self.spi_dev.close()
        except Exception:
            pass
        try:
            os.close(self._lock_fd)      # releases the flock
        except Exception:
            pass

    def _write(self, register, value):
        self.spi_dev.xfer2([register | 0x80, value])

    def _read(self, register, length=1):
        result = [self.spi_dev.xfer2([register + x, 0])[1] for x in range(length)]
        return result[0] if length == 1 else result

    def _bulk_write(self, data):
        for x in range(0, len(data), 2):
            register, value = data[x:x + 2]
            if register == WAIT:
                time.sleep(value / 1000)
            else:
                self._write(register, value)

    def _secret_sauce(self):
        """PixArt PAA5100 init, verbatim from pimoroni/pmw3901-python."""
        self._bulk_write([
            0x7F, 0x00,
            0x55, 0x01,
            0x50, 0x07,

            0x7F, 0x0E,
            0x43, 0x10
        ])
        if self._read(0x67) & 0b10000000:
            self._write(0x48, 0x04)
        else:
            self._write(0x48, 0x02)
        self._bulk_write([
            0x7F, 0x00,
            0x51, 0x7B,
            0x50, 0x00,
            0x55, 0x00,
            0x7F, 0x0E
        ])
        if self._read(0x73) == 0x00:
            c1 = self._read(0x70)
            c2 = self._read(0x71)
            if c1 <= 28:
                c1 += 14
            if c1 > 28:
                c1 += 11
            c1 = max(0, min(0x3F, c1))
            c2 = (c2 * 45) // 100
            self._bulk_write([
                0x7F, 0x00,
                0x61, 0xAD,
                0x51, 0x70,
                0x7F, 0x0E
            ])
            self._write(0x70, c1)
            self._write(0x71, c2)
        self._bulk_write([
            0x7F, 0x00,
            0x61, 0xAD,

            0x7F, 0x03,
            0x40, 0x00,

            0x7F, 0x05,
            0x41, 0xB3,
            0x43, 0xF1,
            0x45, 0x14,

            0x5F, 0x34,
            0x7B, 0x08,
            0x5E, 0x34,
            0x5B, 0x11,
            0x6D, 0x11,
            0x45, 0x17,
            0x70, 0xE5,
            0x71, 0xE5,

            0x7F, 0x06,
            0x44, 0x1B,
            0x40, 0xBF,
            0x4E, 0x3F,

            0x7F, 0x08,
            0x66, 0x44,
            0x65, 0x20,
            0x6A, 0x3A,
            0x61, 0x05,
            0x62, 0x05,

            0x7F, 0x09,
            0x4F, 0xAF,
            0x5F, 0x40,
            0x48, 0x80,
            0x49, 0x80,
            0x57, 0x77,
            0x60, 0x78,
            0x61, 0x78,
            0x62, 0x08,
            0x63, 0x50,

            0x7F, 0x0A,
            0x45, 0x60,

            0x7F, 0x00,
            0x4D, 0x11,
            0x55, 0x80,
            0x74, 0x21,
            0x75, 0x1F,
            0x4A, 0x78,
            0x4B, 0x78,
            0x44, 0x08,

            0x45, 0x50,
            0x64, 0xFF,
            0x65, 0x1F,

            0x7F, 0x14,
            0x65, 0x67,
            0x66, 0x08,
            0x63, 0x70,
            0x6F, 0x1C,

            0x7F, 0x15,
            0x48, 0x48,

            0x7F, 0x07,
            0x41, 0x0D,
            0x43, 0x14,
            0x4B, 0x0E,
            0x45, 0x0F,
            0x44, 0x42,
            0x4C, 0x80,

            0x7F, 0x10,
            0x5B, 0x02,

            0x7F, 0x07,
            0x40, 0x41,

            WAIT, 0x0A,  # Wait 10ms

            0x7F, 0x00,
            0x32, 0x00,

            0x7F, 0x07,
            0x40, 0x40,

            0x7F, 0x06,
            0x68, 0xF0,
            0x69, 0x00,

            0x7F, 0x0D,
            0x48, 0xC0,
            0x6F, 0xD5,

            0x7F, 0x00,
            0x5B, 0xA0,
            0x4E, 0xA8,
            0x5A, 0x90,
            0x40, 0x80,
            0x73, 0x1F,

            WAIT, 0x0A,  # Wait 10ms

            0x73, 0x00
        ])
