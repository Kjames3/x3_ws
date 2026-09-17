"""PAA5100JE burst parsing, axis mapping and velocity math -- no hardware, no ROS.

Calibration (2026-09-16, sensor lens 26.5 mm above the floor):

  * axes: robot forward = +dy, robot left = +dx.  No mirror, no swap beyond
    that, with the vendored init (which never writes REG_ORIENTATION).
  * scale: 23,850 counts/m, from summed raw sensor counts on a 3.000 m
    forward drive on carpet (71,533 counts, tape endpoint within 1 cm,
    0.15 m/s mean, SQUAL >= 189).  24,400 from earlier 1 m drives is void: it
    integrated /flow/twist in a compare script whose subscriber dropped
    messages.  ~26,000 from 20 cm hand pushes WAS raw counts, but short and
    probably with the chassis pressed down (lower sensor, more counts).
    The left axis is NOT separately calibrated: strafes arced 16-21 deg.
    The scale is height-dependent (and may differ between carpet and hard
    floor): re-measure after any mount change.

The trap this module exists to guard: on shiny / low-texture strips the chip
keeps asserting data-ready and keeps counting, but UNDER-counts by 15-20%.
Nothing in the motion bit says so.  The only tell is SQUAL collapsing together
with the shutter, so trust has to be derived from SQUAL, not from data-ready.
"""
import math
import struct
from dataclasses import dataclass

DEFAULT_COUNTS_PER_M = 23850.0

# Pimoroni's own reject rule: very low SQUAL with the shutter pegged means the
# chip has no usable image at all (lifted off the floor, or pitch black).
REJECT_SQUAL = 0x19
REJECT_SHUTTER_UPPER = 0x1F

# SQUAL on bare floor at 26.5 mm sits at 200-225; over tape it fell to 75-140
# while under-counting.  Below GOOD_SQUAL the covariance is inflated.
GOOD_SQUAL = 150


@dataclass(frozen=True)
class Burst:
    motion: bool      # data-ready / motion bit
    dx: int           # counts since the previous burst read
    dy: int
    squal: int        # surface quality, 0-255
    raw_sum: int
    raw_max: int
    raw_min: int
    shutter: int      # 16-bit exposure

    @property
    def rejected(self):
        return self.squal < REJECT_SQUAL and (self.shutter >> 8) == REJECT_SHUTTER_UPPER


def parse_burst(data):
    """12 bytes from REG_MOTION_BURST (after the address echo) -> Burst."""
    if len(data) != 12:
        raise ValueError(f'motion burst must be 12 bytes, got {len(data)}')
    dr, _obs, dx, dy, squal, rsum, rmax, rmin, su, sl = struct.unpack('<BBhhBBBBBB', bytes(data))
    motion = bool(dr & 0x80)
    if not motion:
        # The delta registers are only meaningful when the motion bit is set.
        dx = dy = 0
    return Burst(motion, dx, dy, squal, rsum, rmax, rmin, (su << 8) | sl)


def counts_to_sensor_velocity(dx, dy, dt, counts_per_m=DEFAULT_COUNTS_PER_M):
    """Chip counts over dt seconds -> floor velocity AT THE SENSOR, robot axes.

    Returns (vx forward, vy left) in m/s.
    """
    if dt <= 0.0:
        raise ValueError('dt must be positive')
    return dy / counts_per_m / dt, dx / counts_per_m / dt


def sensor_to_body_velocity(vx_s, vy_s, wz, mount_x, mount_y):
    """Remove the rotation-induced flow at an off-centre sensor.

    A point at r = (mount_x, mount_y) from the base_link origin moves at
    v_body + wz x r = (vx - wz*y, vy + wz*x), so the body velocity is the
    measurement minus that term.
    """
    return vx_s + wz * mount_y, vy_s - wz * mount_x


def velocity_variance(speed, squal, *, floor_sigma=0.005, rel_sigma=0.05,
                      good_squal=GOOD_SQUAL, low_squal_factor=10.0):
    """Per-axis variance (m/s)^2 for a velocity estimate.

    floor_sigma covers quantisation and hand-calibration of a ~0 reading;
    rel_sigma the +-3% scale scatter plus height changes.  Below good_squal
    the sigma grows linearly to low_squal_factor x at SQUAL 0, since that is
    the regime where the chip was measured under-counting.
    """
    sigma = math.hypot(floor_sigma, rel_sigma * abs(speed))
    if squal < good_squal:
        frac = 1.0 - max(0, squal) / good_squal
        sigma *= 1.0 + (low_squal_factor - 1.0) * frac
    return sigma * sigma
