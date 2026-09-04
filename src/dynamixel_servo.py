#!/usr/bin/env python3
"""Minimal Robotis X-series (Protocol 2.0) driver for the XL430-W250-T.

Unlike the LX-16A (see lx16a_servo.py), Dynamixel framing/checksums are not
hand-rolled here -- Robotis's own `dynamixel_sdk` (pip install dynamixel-sdk)
does that. This module only wraps the control-table addresses this project
needs, the same way lx16a_servo.py wraps LX-16A command ids.

Transport: the OpenRB-150 is a SAMD21 board with a built-in TTL half-duplex
level shifter for the X-series bus, and ships pre-flashed with firmware that
passes DYNAMIXEL Protocol 2.0 straight through from USB to the servo bus --
the same role a U2D2 plays. That means dynamixel_sdk on the Jetson talks to
the XL430 directly over the OpenRB-150's USB-serial port; nothing needs to be
programmed onto the board for this to work.

UNVERIFIED until the hardware is in hand and tested:
  - That the OpenRB-150 is still in passthrough mode out of the box (Robotis
    ships it that way for DYNAMIXEL Wizard 2.0 compatibility, but if a
    standalone Arduino sketch was ever uploaded to it, passthrough is gone
    until the "Passthrough"/wizard-compatible example is reflashed from the
    Arduino IDE -- File > Examples > OpenRB-150).
  - The default baud rate (57600, Robotis's factory default for X-series) and
    default ID (1) -- reset with DYNAMIXEL Wizard 2.0 if unknown.
  - Present Load (register 126) on the XL430 is a PWM-based estimate, not a
    real current-sensor reading (unlike the XL330, which has one) -- treat it
    as directional/relative, not calibrated torque.

Positions are raw servo counts: 0..4095 spans 360 degrees (11.375 counts/deg).
That is a different scale from both the LX-16A (4.1667 counts/deg) and the
Yahboom YB-SD15M Rosmaster_Lib assumes (12.2 counts/deg) -- never mix them.
"""

import time

try:
    from dynamixel_sdk import (PortHandler, PacketHandler,
                                COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE,
                                DXL_LOWORD, DXL_HIWORD)
except ImportError:  # keeps --help usable on machines without dynamixel_sdk
    PortHandler = None
    PacketHandler = None
    COMM_SUCCESS = 0

DEFAULT_PORT = "/dev/openrb150"
# 1 Mbps, written into the servo's EEPROM 2026-08-29 (baud code 3) by
# dynamixel_set_baud.py.  NOT the Robotis factory default of 57600 -- a fresh
# or factory-reset servo will NOT answer here, so use `find_servo()` (it sweeps
# BAUD_SCAN_ORDER) rather than assuming this constant when a servo goes silent.
# Measured round trip for one read_pos: 6.14 ms at 57600, 1.49 ms at 1 Mbps.
# The remaining ~1.4 ms is USB/CDC-ACM turnaround on the OpenRB-150 bridge, not
# the wire, so raising the rate further buys almost nothing.  It matters
# because dynamixel_sdk BUSY-WAITS on the read: latency here is ~97% burned
# CPU, not idle time, so the tilt publisher's cost scales directly with it.
BAUD = 1000000
PROTOCOL_VERSION = 2.0

COUNTS_PER_DEG = 4096.0 / 360.0   # 11.3778
DEG_PER_COUNT = 360.0 / 4096.0    # 0.0879
COUNTS_PER_REV = 4096

# control table addresses (X-series, e.g. XL430/XM430/XL330 share this table)
ADDR_MODEL_NUMBER = 0           # 2 bytes
ADDR_FIRMWARE_VERSION = 6       # 1 byte
ADDR_ID = 7                     # 1 byte, EEPROM
ADDR_BAUD_RATE = 8              # 1 byte, EEPROM (code, not a literal baud)
ADDR_RETURN_DELAY_TIME = 9      # 1 byte
ADDR_OPERATING_MODE = 11        # 1 byte
ADDR_HOMING_OFFSET = 20         # 4 bytes, signed
ADDR_TEMPERATURE_LIMIT = 31     # 1 byte
ADDR_MIN_POSITION_LIMIT = 52    # 4 bytes
ADDR_MAX_POSITION_LIMIT = 48    # 4 bytes
ADDR_TORQUE_ENABLE = 64         # 1 byte
ADDR_HARDWARE_ERROR_STATUS = 70  # 1 byte
ADDR_PROFILE_ACCELERATION = 108  # 4 bytes
ADDR_PROFILE_VELOCITY = 112     # 4 bytes
ADDR_GOAL_POSITION = 116        # 4 bytes, signed
ADDR_MOVING = 122               # 1 byte
ADDR_PRESENT_LOAD = 126         # 2 bytes, signed, PWM-based estimate on XL430
ADDR_PRESENT_VELOCITY = 128     # 4 bytes, signed
ADDR_PRESENT_POSITION = 132     # 4 bytes, signed
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, 0.1 V units
ADDR_PRESENT_TEMPERATURE = 146  # 1 byte, deg C

OPERATING_MODE_POSITION = 3     # single-turn position control (0-4095)

BROADCAST_ID = 254
MAX_ID = 252

# Baud Rate register codes (X-series). The register stores the code, not the
# rate, so a servo "at 57600" really has an 8-bit 1 in EEPROM address 8.
BAUD_CODES = {0: 9600, 1: 57600, 2: 115200, 3: 1000000,
              4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000}
# Order to sweep when hunting for an unknown servo: factory default first,
# then the two rates a previous owner is most likely to have set.
BAUD_SCAN_ORDER = (57600, 1000000, 115200, 2000000, 9600, 3000000,
                   4000000, 4500000)

# Model Number (register 0) -> name. Only what might plausibly turn up on this
# bench; an unlisted number is reported as a raw number, not guessed at.
MODEL_NUMBERS = {
    1060: "XL430-W250-T",
    1090: "2XL430-W250-T",
    1070: "XC430-W150-T",
    1080: "XC430-W240-T",
    1020: "XM430-W350-T",
    1030: "XM430-W210-T",
    1200: "XL330-M288-T",
    1190: "XL330-M077-T",
    350: "XL-320",
    12: "AX-12A",
}
EXPECTED_MODEL = 1060           # what this project is wired for


def model_name(number):
    return MODEL_NUMBERS.get(number, "unknown model %s" % number)


def find_ports():
    """Candidate serial ports for an OpenRB-150, best guess first.

    The udev symlink (see src/64-openrb150.rules) is preferred, but that rule
    is still a template until the board's real VID/PID is read off the bench,
    so fall back to raw ACM/USB nodes.
    """
    import glob
    import os
    # If udev has published the symlink, that IS the board -- do not go on to
    # sweep /dev/ttyUSB*, which on this robot is the YDLidar at 512000 baud.
    # Poking it with Protocol 2.0 pings is pointless and slows identify down.
    if os.path.exists("/dev/openrb150"):
        return ["/dev/openrb150"]
    ports = []
    seen = set()
    # Deduplicate by the node the name resolves to: once 64-openrb150.rules is
    # installed, /dev/openrb150 and /dev/ttyACM0 are the SAME device, and
    # probing both opens the servo bus twice ("device reports readiness to
    # read but returned no data"). The symlink is listed first so it wins.
    for pattern in ("/dev/openrb150", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            real = os.path.realpath(path)
            if real in seen:
                continue
            seen.add(real)
            ports.append(path)
    return ports

# Profile Velocity units: 0.229 rev/min per count in position-control mode.
# 1 rev/min = 6 deg/s, so deg/s -> counts is deg_s / 6 / 0.229.
_RPM_PER_COUNT = 0.229
_DEG_S_PER_RPM = 6.0


def deg_s_to_profile_velocity(deg_s):
    if deg_s <= 0:
        return 0  # 0 = firmware max velocity, not "stopped"
    return max(1, int(round(deg_s / _DEG_S_PER_RPM / _RPM_PER_COUNT)))


def find_servo(servo_id=1, port=DEFAULT_PORT, bauds=BAUD_SCAN_ORDER):
    """Open the bus at whatever rate the servo is actually answering on.

    Returns (XL430, baud).  This is the recovery path after a baud change: the
    rate lives in the servo's EEPROM, so a half-finished change leaves the
    hardware fine but the hardcoded default wrong, which presents as "the
    servo is gone".  Sweeping is cheap (one ping per rate) and unambiguous.
    """
    last = None
    for baud in bauds:
        try:
            dev = XL430(port, baud=baud)
        except DynamixelError as e:
            last = e
            continue
        try:
            if dev.ping(servo_id) is not None:
                return dev, baud
        except DynamixelError as e:
            last = e
        dev.close()
    raise DynamixelError(
        "servo id %d did not answer on %s at any of %s (last error: %s)"
        % (servo_id, port, list(bauds), last))


class DynamixelError(Exception):
    pass


class XL430:
    """Blocking, synchronous client for a single X-series servo bus."""

    def __init__(self, port=DEFAULT_PORT, baud=BAUD, protocol=PROTOCOL_VERSION):
        if PortHandler is None:
            raise DynamixelError("dynamixel_sdk is not installed "
                                  "(pip3 install dynamixel-sdk)")
        self.port = port
        self._port_handler = PortHandler(port)
        self._packet = PacketHandler(protocol)
        if not self._port_handler.openPort():
            raise DynamixelError("cannot open %s" % port)
        if not self._port_handler.setBaudRate(baud):
            self._port_handler.closePort()
            raise DynamixelError("cannot set baud %d on %s" % (baud, port))

    def close(self):
        try:
            self._port_handler.closePort()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- low-level table access ----------------------------------------
    def _read(self, servo_id, addr, size):
        if size == 1:
            fn = self._packet.read1ByteTxRx
        elif size == 2:
            fn = self._packet.read2ByteTxRx
        elif size == 4:
            fn = self._packet.read4ByteTxRx
        else:
            raise ValueError("unsupported read size %d" % size)
        value, comm, err = fn(self._port_handler, servo_id, addr)
        if comm != COMM_SUCCESS:
            raise DynamixelError("read addr %d id %d: %s" % (
                addr, servo_id, self._packet.getTxRxResult(comm)))
        if err != 0:
            raise DynamixelError("read addr %d id %d: servo error 0x%02x" % (
                addr, servo_id, err))
        return value

    def _write(self, servo_id, addr, size, value):
        if size == 1:
            fn = self._packet.write1ByteTxRx
        elif size == 2:
            fn = self._packet.write2ByteTxRx
        elif size == 4:
            fn = self._packet.write4ByteTxRx
        else:
            raise ValueError("unsupported write size %d" % size)
        comm, err = fn(self._port_handler, servo_id, addr, value)
        if comm != COMM_SUCCESS:
            raise DynamixelError("write addr %d id %d: %s" % (
                addr, servo_id, self._packet.getTxRxResult(comm)))
        if err != 0:
            raise DynamixelError("write addr %d id %d: servo error 0x%02x" % (
                addr, servo_id, err))

    @staticmethod
    def _to_signed32(v):
        return v - (1 << 32) if v > 0x7FFFFFFF else v

    @staticmethod
    def _to_signed16(v):
        return v - (1 << 16) if v > 0x7FFF else v

    # ---- reads -----------------------------------------------------------
    def ping(self, servo_id):
        model, comm, err = self._packet.ping(self._port_handler, servo_id)
        if comm != COMM_SUCCESS:
            return None
        return model

    def broadcast_ping(self):
        """Ask every id on the bus to answer at once -> {id: (model, fw)}.

        Far faster than scan(), but some USB bridges swallow the burst of
        replies; treat an empty result as inconclusive and fall back to scan().
        """
        found, comm = self._packet.broadcastPing(self._port_handler)
        if comm != COMM_SUCCESS:
            return {}
        return {i: (v[0], v[1]) for i, v in found.items()}

    def read_model(self, servo_id):
        return self._read(servo_id, ADDR_MODEL_NUMBER, 2)

    def read_firmware(self, servo_id):
        return self._read(servo_id, ADDR_FIRMWARE_VERSION, 1)

    def read_baud_code(self, servo_id):
        return self._read(servo_id, ADDR_BAUD_RATE, 1)

    def read_homing_offset(self, servo_id):
        return self._to_signed32(self._read(servo_id, ADDR_HOMING_OFFSET, 4))

    def read_pos(self, servo_id):
        return self._to_signed32(self._read(servo_id, ADDR_PRESENT_POSITION, 4))

    def read_velocity(self, servo_id):
        return self._to_signed32(self._read(servo_id, ADDR_PRESENT_VELOCITY, 4))

    def read_load(self, servo_id):
        return self._to_signed16(self._read(servo_id, ADDR_PRESENT_LOAD, 2))

    def read_temp(self, servo_id):
        return self._read(servo_id, ADDR_PRESENT_TEMPERATURE, 1)

    def read_vin(self, servo_id):
        return self._read(servo_id, ADDR_PRESENT_INPUT_VOLTAGE, 2) / 10.0

    def read_moving(self, servo_id):
        return bool(self._read(servo_id, ADDR_MOVING, 1))

    def read_hardware_error(self, servo_id):
        return self._read(servo_id, ADDR_HARDWARE_ERROR_STATUS, 1)

    def read_position_limits(self, servo_id):
        return (self._read(servo_id, ADDR_MIN_POSITION_LIMIT, 4),
                self._read(servo_id, ADDR_MAX_POSITION_LIMIT, 4))

    def is_loaded(self, servo_id):
        """True when torque is enabled (holding/able to move)."""
        return bool(self._read(servo_id, ADDR_TORQUE_ENABLE, 1))

    def scan(self, ids=range(0, 253)):
        found = []
        for i in ids:
            if self.ping(i) is not None:
                found.append(i)
        return found

    # ---- writes ------------------------------------------------------
    def set_load(self, servo_id, on):
        """Torque enable/disable. Must be OFF to change limits/mode."""
        self._write(servo_id, ADDR_TORQUE_ENABLE, 1, 1 if on else 0)

    def set_baud_code(self, servo_id, code):
        """Rewrite the servo's baud rate (EEPROM). Torque must be off.

        `code` is a BAUD_CODES key, NOT a literal rate -- register 8 stores 3
        for 1 Mbps.  The servo switches the instant the write is acknowledged,
        so this connection is dead afterwards: close the port and reopen at the
        NEW rate (see `find_servo`, which sweeps for exactly this reason).

        If the reopen fails the servo is not bricked, only unreachable at the
        old rate -- `find_servo` will still locate it because BAUD_SCAN_ORDER
        covers every code in the table.
        """
        if code not in BAUD_CODES:
            raise ValueError("baud code %r is not in %s"
                             % (code, sorted(BAUD_CODES)))
        self._write(servo_id, ADDR_BAUD_RATE, 1, int(code))

    def set_id(self, servo_id, new_id):
        """Rewrite the servo's bus id (EEPROM). Torque must be off.

        After this returns the servo answers only to `new_id`; the caller is
        responsible for not addressing the old one again.
        """
        if not 0 <= new_id <= MAX_ID:
            raise ValueError("id must be 0..%d" % MAX_ID)
        self._write(servo_id, ADDR_ID, 1, new_id)

    def set_homing_offset(self, servo_id, counts):
        """EEPROM position offset: Present Position = actual + offset.

        Torque must be off. NOTE this does NOT shift the commandable range --
        Min/Max Position Limit still clamp Goal Position to 0..4095 in the
        offset frame, so a large offset silently costs you travel at one end.
        dynamixel_tilt.py keeps zero in the calibration file instead, which
        costs nothing; see src/dynamixel_setup.py --write-homing-offset.
        """
        self._write(servo_id, ADDR_HOMING_OFFSET, 4, int(counts) & 0xFFFFFFFF)

    def set_operating_mode(self, servo_id, mode=OPERATING_MODE_POSITION):
        """Torque must be off before this will take (servo NACKs otherwise)."""
        self._write(servo_id, ADDR_OPERATING_MODE, 1, mode)

    def set_profile_velocity(self, servo_id, counts):
        """0 = firmware max speed; otherwise 0.229 rev/min per count."""
        self._write(servo_id, ADDR_PROFILE_VELOCITY, 4, int(max(0, counts)))

    def set_profile_acceleration(self, servo_id, counts):
        self._write(servo_id, ADDR_PROFILE_ACCELERATION, 4, int(max(0, counts)))

    def move(self, servo_id, counts, velocity_counts=None):
        """Write a goal position (0..4095). Non-blocking -- poll read_moving()
        or read_pos() to know when it has arrived."""
        counts = int(max(0, min(COUNTS_PER_REV - 1, counts)))
        if velocity_counts is not None:
            self.set_profile_velocity(servo_id, velocity_counts)
        self._write(servo_id, ADDR_GOAL_POSITION, 4, counts)

    def wait_until_settled(self, servo_id, timeout_s=5.0, poll_s=0.05,
                           start_grace_s=0.5):
        """Block until the servo stops moving.

        The Moving flag (addr 122) does not assert until Present Velocity
        clears Moving Threshold, which is several ms after the Goal Position
        write lands -- a naive "return as soon as it is not moving" poll
        therefore reports success before the servo has begun, and the caller
        commands the next waypoint mid-travel. Ignore a not-moving reading
        until motion has actually been observed, or start_grace_s has passed
        (which covers a goal the servo was already sitting on, and slow
        profiles that never clear Moving Threshold at all).
        """
        t0 = time.monotonic()
        started = False
        while time.monotonic() - t0 < timeout_s:
            if self.read_moving(servo_id):
                started = True
            elif started or (time.monotonic() - t0) >= start_grace_s:
                return True
            time.sleep(poll_s)
        return False
