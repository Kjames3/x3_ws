# Dual ToF Teensy bench test and USB transport

Upload `teensy_tof_test.ino` with Board **Teensy 4.1**, USB Type **Serial**, and
SparkFun VL53L5CX Arduino Library (compiled here with 1.0.3 / Teensy 1.62.0).
This firmware reads two VL53L5CX sensors at 8x8, 15 Hz, continuous mode,
1 MHz I2C, sharpener 5% (matching the existing Jetson driver).
It does not move the robot. Two-sensor hardware validation is still pending.

## Wiring (power off while connecting)

| Sensor pin | Upper sensor | Lower sensor |
|---|---|---|
| SDA | Teensy 18 (`Wire`) | Teensy 17 (`Wire1`) |
| SCL | Teensy 19 | Teensy 16 |
| VIN | External regulated 3.3 V | Same external regulated 3.3 V |
| GND | Common supply/Teensy GND | Common supply/Teensy GND |
| LPn | Sensor supply 3.3 V | Sensor supply 3.3 V |
| INT | Unconnected | Unconnected |

The pin numbers are Teensy labels, not physical header positions. Each sensor
has its own I2C bus and retains address 0x29; no address-assignment sequence is
needed. Existing pullups on each breakout should go to 3.3 V. Keep leads short.
The laptop or Jetson connects to the Teensy's normal USB device connector.
Do not also connect the sensor I2C lines to the Jetson.

**Use a regulated 3.3 V sensor supply rated for at least 500 mA** and suitable
for transient loads, with local decoupling at each breakout. Connect supply
GND to Teensy GND. Keep its positive output separate from the Teensy's 3.3 V
output; the Teensy remains USB-powered. Do not feed the breakouts' signal pins
with 5 V. Power both systems for operation rather than leaving a powered I2C
bus attached to an unpowered participant.

Why a separate supply: PJRC recommends at most 250 mA external load on Teensy's
3.3 V output. ST lists active-ranging maximum average currents of 50 mA AVDD
plus 80 mA IOVDD per sensor, with peak current above those averages. Two sensor
modules plus breakout circuitry leave insufficient worst-case margin. Their
actual VIN current depends on the breakout power circuit. This recommendation
does not imply the earlier one-sensor test was faulty.

For the user's spare LM2596S module: battery positive -> IN+, battery negative
-> IN-. Set and verify OUT+ relative to OUT- at 3.3 V with a meter before
connecting sensors. OUT+ feeds both sensor VIN and LPn pins. OUT- connects
both sensor GND pins and a Teensy GND pin. Teensy stays powered by USB;
do not connect buck OUT+ to Teensy VIN or 3.3 V. The dedicated common-ground
wire provides the I2C reference without relying on a ground path through the
Jetson's USB/power wiring. Check the output again under the two-sensor load.
The exact module's condition, adjustment, and ripple have not been tested.

References: [PJRC power](https://www.pjrc.com/store/teensy41.html),
[PJRC I2C pins](https://www.pjrc.com/teensy/td_libs_Wire.html),
[ST datasheet, section 6.4](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf).
INT is a data-ready output, optional with polling. LPn controls I2C access;
it would be used for address assignment if both sensors shared one bus.

## First check in Arduino

Open Serial Monitor at 115200 and allow both firmware uploads to the sensors
to complete (several seconds). Each sensor emits a status record. A missing
sensor is reported inactive without disabling the other sensor; after fixing
wiring, reset/restart the Teensy to retry initialization.

Raw JSON frames stream by default. Send **b** for benchmark summaries only,
or **r** to resume raw frames. Every five seconds, each sensor reports its
fps, read time, mean valid-zone count, center-zone mean/standard deviation,
error count, transmit-drop count, and age of the last successful read.
The first four frames from each sensor are skipped. Status 5 with a detected
target is the strict validity rule for benchmark statistics. Raw frames retain
all statuses so the host can choose its own filter. Sharpener is explicitly 5%,
so distance-noise comparisons with the older sketch's library default need a
fresh stationary-target baseline.

Check upper/lower identity by presenting a target to one sensor at a time.
Then run both against a stationary wall and check frame rate, errors, validity,
and USB drops. Separate buses do not synchronize acquisition or prevent optical
interaction. Firmware reads the two buses sequentially; at the measured 14.65 ms
per read, two reads occupy about 29.3 ms of a 66.7 ms frame period. Confirm this
with the two-sensor hardware, rather than assuming both rates from one test.

## Receive on laptop or Jetson

Close Arduino Serial Monitor before opening this receiver. Install pyserial
if needed (`python3 -m pip install pyserial`). Prefer the Teensy's stable path
under `/dev/serial/by-id/`; `/dev/ttyACM0` is fine if verified to be the Teensy.

```bash
python3 src/teensy_tof_serial.py --port /dev/ttyACM0 --seconds 30 --output /tmp/tof-dual.jsonl
```

The output file must not already exist. Diagnostics and received frame rates
are printed to stderr. Exit code 2 indicates that at least one sensor produced
no valid protocol frames; invalid optical zones still count as received frames.
The reader does not open any robot controller or automatically change ROS nodes.
A USB disconnect exits clearly; rerun after reconnecting. An initial partial
line is discarded, and subsequent complete lines are recovered.

## Wire format: NDJSON version 1

Every line is a JSON object with `v:1`, `type`, and `sensor:"upper"|"lower"`.
Record types are `status`, `stats`, and `frame`. Frame fields:

- `seq`: per-sensor successful frame-read counter after warmup (uint32).
- `t_ms`: Teensy `millis()` at read completion (uint32, wraps ~49.7 days).
- `read_us`: duration of the frame I2C read, excluding ready polling and USB.
- `distance_mm`: 64 signed millimetre depths, first target in each zone.
- `target_status`: 64 corresponding ST status bytes.
- `nb_target_detected`: 64 target counts; zero means no detected target.

Arrays use native ULD order. No image flips, rotations, spatial transforms,
range gates, or floor rejection are applied in firmware. A zero target count
or invalid status must not become a valid obstacle/free-space reading.

The MCU clock is not synchronized to the Jetson; `t_ms` is neither ROS time
nor the exposure timestamp. `seq` tracks successful reads, not every internal
sensor exposure. Its gaps show omitted USB frame records, not all sensor-side
missed acquisitions. Errors and tx_drops are cumulative since boot. Summaries
use five-second windows. Output queues are bounded; whole new frames may be
omitted if a previous record is still pending, and tx_drops counts those cases.
USB output from different sensors is never interleaved within a JSON record.

This receiver establishes and records the Jetson USB data path. Integration
into x3_server/ROS, timestamp alignment, and the two new URDF sensor poses are
a subsequent step after the physical wiring and stream pass bench validation.
