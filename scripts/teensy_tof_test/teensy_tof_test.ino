/* Dual VL53L5CX -> Teensy 4.1 -> USB Serial (newline-delimited JSON v1).
 * Upper: SDA 18, SCL 19 (Wire). Lower: SDA 17, SCL 16 (Wire1).
 * Both LPn high, INT unused. Both retain 7-bit address 0x29.
 * See README.md for power wiring. Library: SparkFun VL53L5CX Arduino Library.
 * Board: Teensy 4.1, USB Type: Serial. No automatic reset/reflash.
 * Commands: b = benchmark summaries only; r = resume raw frames (default).
 * Raw distances/statuses remain in ULD order, first target per zone.
 */
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <math.h>
#include <stdarg.h>

constexpr uint32_t I2C_HZ = 1000000;
constexpr uint8_t FRAME_HZ = 15;
constexpr uint32_t REPORT_MS = 5000;
constexpr uint8_t CENTER_ZONE = 27;

struct Channel {
  const char *id;
  TwoWire *bus;
  SparkFun_VL53L5CX sensor;
  VL53L5CX_ResultsData data;
  bool active = false;
  uint8_t warmup = 4;
  uint32_t seq = 0, errors = 0, txDrops = 0;
  uint32_t startMs = 0, lastMs = 0, frames = 0, readUs = 0, maxReadUs = 0;
  uint32_t valid = 0, centerN = 0;
  double mean = 0, m2 = 0;
  char tx[2048];
  size_t length = 0, offset = 0;
  Channel(const char *name, TwoWire &wire) : id(name), bus(&wire) {}
};
Channel upper("upper", Wire), lower("lower", Wire1), clockReply("teensy", Wire);
Channel *channels[] = {&upper, &lower};
Channel *usbChannels[] = {&clockReply, &upper, &lower};
bool readingSync = false, syncPending = false;
uint32_t syncToken = 0, syncRxMs = 0;
unsigned syncDigits = 0;
bool streamFrames = true;

// Two callbacks keep transport errors attributed to the correct sensor.
void upperError(SF_VL53L5CX_ERROR_TYPE, uint32_t) { ++upper.errors; }
void lowerError(SF_VL53L5CX_ERROR_TYPE, uint32_t) { ++lower.errors; }

void append(Channel &c, const char *format, ...) {
  if (c.length >= sizeof(c.tx)) return;
  va_list args;
  va_start(args, format);
  int n = vsnprintf(c.tx + c.length, sizeof(c.tx) - c.length, format, args);
  va_end(args);
  if (n < 0 || size_t(n) >= sizeof(c.tx) - c.length) c.length = sizeof(c.tx);
  else c.length += n;
}

void finishLine(Channel &c) {
  // Never transmit a truncated JSON record.
  if (c.length >= sizeof(c.tx)) { c.length = c.offset = 0; ++c.txDrops; }
}

void event(Channel &c, const char *message) {
  c.length = c.offset = 0;
  append(c, "{\"v\":1,\"type\":\"status\",\"sensor\":\"%s\",\"active\":%s,\"message\":\"%s\"}\n",
         c.id, c.active ? "true" : "false", message);
  finishLine(c);
}

bool good(Channel &c, unsigned z) {
  return c.data.nb_target_detected[z] > 0 &&
         c.data.target_status[z * VL53L5CX_NB_TARGET_PER_ZONE] == 5;
}

void initialize(Channel &c) {
  c.bus->begin();
  c.bus->setClock(I2C_HZ);
  c.active = c.sensor.begin(0x29, *c.bus);
  if (c.active) c.active = c.sensor.setResolution(64);
  if (c.active) c.active = c.sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  if (c.active) c.active = c.sensor.setRangingFrequency(FRAME_HZ);
  if (c.active) c.active = c.sensor.setSharpenerPercent(5); // Match Jetson driver.
  // Start both only after both firmware uploads finish.
}

void setup() {
  Serial.begin(115200); // USB CDC; this number does not limit USB throughput.
  while (!Serial && millis() < 3000) delay(10);
  upper.sensor.setErrorCallback(upperError);
  lower.sensor.setErrorCallback(lowerError);
  for (Channel *c : channels) initialize(*c);
  for (Channel *c : channels) {
    if (c->active) c->active = c->sensor.startRanging();
    c->startMs = millis();
    event(*c, c->active ? "ranging" : "init_failed_check_wiring_then_reset");
  }
}

void queueFrame(Channel &c, uint32_t elapsed) {
  if (!streamFrames || !Serial) return;
  if (c.length) { ++c.txDrops; return; } // Preserve pending record, never splice.
  append(c, "{\"v\":1,\"type\":\"frame\",\"sensor\":\"%s\",\"seq\":%lu,\"t_ms\":%lu,\"read_us\":%lu,\"distance_mm\":[",
         c.id, (unsigned long)c.seq, (unsigned long)c.lastMs, (unsigned long)elapsed);
  for (unsigned z = 0; z < 64; ++z)
    append(c, "%s%d", z ? "," : "", int(c.data.distance_mm[z * VL53L5CX_NB_TARGET_PER_ZONE]));
  append(c, "],\"target_status\":[");
  for (unsigned z = 0; z < 64; ++z)
    append(c, "%s%u", z ? "," : "", unsigned(c.data.target_status[z * VL53L5CX_NB_TARGET_PER_ZONE]));
  append(c, "],\"nb_target_detected\":[");
  for (unsigned z = 0; z < 64; ++z)
    append(c, "%s%u", z ? "," : "", unsigned(c.data.nb_target_detected[z]));
  append(c, "]}\n");
  finishLine(c);
}

void sample(Channel &c) {
  if (!c.active || !c.sensor.isDataReady()) return;
  uint32_t beginUs = micros();
  if (!c.sensor.getRangingData(&c.data)) return;
  uint32_t elapsed = micros() - beginUs;
  c.lastMs = millis();
  if (c.warmup) { --c.warmup; c.startMs = c.lastMs; return; }
  ++c.seq; ++c.frames; c.readUs += elapsed;
  if (elapsed > c.maxReadUs) c.maxReadUs = elapsed;
  for (unsigned z = 0; z < 64; ++z) c.valid += good(c, z);
  if (good(c, CENTER_ZONE)) {
    double mm = c.data.distance_mm[CENTER_ZONE * VL53L5CX_NB_TARGET_PER_ZONE];
    ++c.centerN;
    double d = mm - c.mean;
    c.mean += d / c.centerN; c.m2 += d * (mm - c.mean);
  }
  queueFrame(c, elapsed);
}

void report(Channel &c) {
  uint32_t now = millis();
  if (now - c.startMs < REPORT_MS || c.length || !Serial) return;
  append(c, "{\"v\":1,\"type\":\"stats\",\"sensor\":\"%s\",\"active\":%s,\"t_ms\":%lu,\"seq\":%lu,\"fps\":%.2f,\"read_ms\":%.2f,\"max_read_ms\":%.2f,\"valid_zones\":%.1f,\"center_n\":%lu,\"center_mm\":",
         c.id, c.active ? "true" : "false", (unsigned long)now, (unsigned long)c.seq,
         c.frames * 1000.0 / (now - c.startMs), c.frames ? c.readUs / (1000.0 * c.frames) : 0,
         c.maxReadUs / 1000.0, c.frames ? double(c.valid) / c.frames : 0, (unsigned long)c.centerN);
  if (c.centerN) append(c, "%.2f", c.mean); else append(c, "null");
  append(c, ",\"center_sd_mm\":");
  if (c.centerN > 1) append(c, "%.2f", sqrt(c.m2 / (c.centerN - 1))); else append(c, "null");
  append(c, ",\"errors\":%lu,\"tx_drops\":%lu,\"last_frame_age_ms\":",
         (unsigned long)c.errors, (unsigned long)c.txDrops);
  if (c.lastMs) append(c, "%lu", (unsigned long)(now - c.lastMs)); else append(c, "null");
  append(c, "}\n"); finishLine(c);
  c.startMs = now; c.frames = c.readUs = c.maxReadUs = c.valid = c.centerN = 0;
  c.mean = c.m2 = 0;
}

// Finish one record before touching the other channel's queue. Nonblocking,
// bounded chunks keep a missing/slow USB reader from stalling sensor polling.
void pumpUSB() {
  static Channel *sending = nullptr;
  if (!Serial) {
    for (Channel *c : usbChannels) c->length = c->offset = 0;
    sending = nullptr;
    return;
  }
  if (!sending) {
    for (Channel *c : usbChannels) if (c->length) { sending = c; break; }
  }
  if (!sending) return;
  for (unsigned k = 0; k < 8 && sending; ++k) {
    size_t count = min(size_t(Serial.availableForWrite()), sending->length - sending->offset);
    count = min(count, size_t(64));
    if (!count) break;
    sending->offset += Serial.write((const uint8_t *)sending->tx + sending->offset, count);
    if (sending->offset == sending->length) {
      sending->length = sending->offset = 0; sending = nullptr;
    }
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { readingSync = true; syncDigits = 0; syncToken = 0; }
    else if (readingSync && c >= '0' && c <= '9' && syncDigits < 9) {
      syncToken = syncToken * 10 + (c - '0'); ++syncDigits;
    } else if (readingSync && c == '\n' && syncDigits) {
      syncRxMs = millis(); syncPending = true; readingSync = false;
    } else {
      readingSync = false;
      if (c == 'b') streamFrames = false;
      if (c == 'r') streamFrames = true;
    }
  }
  if (syncPending && !clockReply.length) {
    append(clockReply, "{\"v\":1,\"type\":\"sync\",\"sensor\":\"teensy\",\"token\":%lu,\"rx_ms\":%lu,\"tx_ms\":%lu}\n",
           (unsigned long)syncToken, (unsigned long)syncRxMs, (unsigned long)millis());
    finishLine(clockReply); syncPending = false;
  }
  pumpUSB();
  for (Channel *c : channels) { report(*c); sample(*c); }
  pumpUSB();
  delay(1);
}
