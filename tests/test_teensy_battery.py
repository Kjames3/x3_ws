"""Teensy battery monitor CSV: parsing, corruption rejection, staleness, time left."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import teensy_battery as tb  # noqa: E402


def test_parses_the_sketch_format():
    s = tb.parse_line(b"82.4,13.104,1.032,16.480,DISCHARGING\r\n", t=5.0)
    assert s == tb.BatterySample(82.4, 13.104, 1.032, 16.48, "DISCHARGING", 5.0)
    assert tb.parse_line("100.0,14.412,-0.640,20.000,CHARGING").state == "CHARGING"
    assert tb.parse_line("50.0,13.180,0.010,10.000,IDLE").state == "IDLE"


@pytest.mark.parametrize("line", [
    "# pct,volts,amps,ah_remaining,state",           # header / diagnostics
    "# boot: EEPROM 81.2%",
    "",
    "82.4,13.104,1.032,16.480",                       # truncated
    "82.4,13.104,1.032,16.480,DISCHARGING,x",         # extra field
    "82.4,13.1O4,1.032,16.480,DISCHARGING",           # letter O for zero
    "82.4,13.104,1.032,16.480,DISCHARGINF",           # bad state
    "82.4,13.104,-1.032,16.480,DISCHARGING",          # dropped/added '-': state disagrees
    "82.4,13.104,0.032,16.480,DISCHARGING",           # flipped digit: 0.03 A is IDLE
    "32.4,13.104,1.032,16.480,DISCHARGING",           # flipped digit: pct vs ah_remaining
    "82.4,93.104,1.032,16.480,DISCHARGING",           # implausible voltage
    "nan,13.104,1.032,16.480,DISCHARGING",
    b"\xff\xfe82.4,13.104,1.032,16.480,DISCHARGING",  # framing garbage
])
def test_rejects_comments_and_corruption(line):
    assert tb.parse_line(line) is None


def test_rejected_lines_are_counted_but_comments_are_not():
    m = tb.TeensyBatteryMonitor(start=False)
    m.feed("# boot")
    m.feed("82.4,13.1O4,1.032,16.480,DISCHARGING")
    m.feed("82.4,13.104,1.032,16.480,DISCHARGING")
    assert (m.lines_ok, m.lines_rejected) == (1, 1)


def test_getters_go_none_when_stale():
    m = tb.TeensyBatteryMonitor(start=False)
    assert m.get_voltage() is None and m.get_percentage() is None
    m.feed("82.4,13.104,1.032,16.480,DISCHARGING", t=100.0)
    s = m.latest(now=101.0)
    assert s.volts == 13.104 and s.pct == 82.4
    assert m.latest(now=100.0 + tb.STALE_S + 0.1) is None


def test_getters_mirror_the_ina226_interface():
    m = tb.TeensyBatteryMonitor(start=False)
    m.feed("82.4,13.104,1.032,16.480,DISCHARGING")
    assert m.get_voltage() == 13.104
    assert m.get_current() == 1.032
    assert m.get_power() == pytest.approx(13.104 * 1.032)
    assert m.get_percentage(12.0) == 82.4        # voltage argument ignored


def test_minutes_remaining_uses_smoothed_discharge_and_resets_on_charge():
    m = tb.TeensyBatteryMonitor(start=False)
    t = 0.0
    for _ in range(40):                            # steady 2 A
        m.feed("50.0,13.180,2.000,10.000,DISCHARGING", t=t)
        t += 0.25
    m.feed("50.0,13.100,8.000,10.000,DISCHARGING", t=t)   # one motor burst
    mins = m.minutes_remaining(now=t)
    assert 290 < mins < 300                        # ~10 Ah / 2 A = 300 min, burst barely moves it
    t += 0.25
    m.feed("50.0,14.000,-3.000,10.000,CHARGING", t=t)
    assert m.minutes_remaining(now=t) is None
    t += 0.25
    m.feed("50.0,13.180,0.050,10.000,IDLE", t=t)
    assert m.minutes_remaining(now=t) is None
