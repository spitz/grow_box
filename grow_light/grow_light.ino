/*
 * grow_light.ino  —  step 2: scheduled light + controllable fan
 *
 * Adds a 4-pin PWM case fan to the light schedule from step 1.
 *
 * WIRING ADDED IN THIS STEP
 *   D9 ---[ 100 R ]--- fan pin 4 (PWM, usually blue)
 *   D2 ----------------- fan pin 3 (tach, usually green)
 *   D2 ---[ 10 k ]----- +5 V          (tach is open collector, needs pull-up)
 *   fan pin 2 --------- +12 V
 *   fan pin 1 --------- GND
 *
 * TIMER NOTE
 *   Timer1 is reconfigured for 25 kHz, which is what the 4-wire fan spec
 *   wants. That breaks analogWrite() on pins 9 and 10 — we write OCR1A
 *   directly instead. Timer0 (millis) and Timer2 (pin 3, the light) are
 *   untouched, so the light channel is unaffected.
 *
 * EEPROM NOTE
 *   The settings struct grew in this version, so your saved settings from
 *   step 1 will not load. The board falls back to defaults and says so.
 *   Re-enter anything you had customised and SAVE again.
 *
 * FAN DUTY IS 0-255
 *   Timer1's native resolution here is 640 steps, but 0-255 keeps the fan
 *   command consistent with LEVEL and the difference is far below anything
 *   the fan can resolve. 0 does not necessarily mean stopped — most 4-pin
 *   fans hold a minimum speed at 0 % duty by design. Run SWEEP to find out
 *   what yours actually does.
 *
 * HEARTBEAT
 *   The onboard LED on D13 breathes continuously so you can see at a glance
 *   that the sketch is alive. D13 has no hardware PWM on the Uno, so this is
 *   bit-banged at 250 Hz in the main loop — which makes it a real liveness
 *   indicator: if loop() ever stalls, the LED freezes rather than carrying on
 *   under its own timer. A slow breath means AUTO, a fast one means you have
 *   left the light forced ON or OFF. Turn it off with HB 0.
 *
 * Type HELP over serial at 115200 baud.
 */

#include <EEPROM.h>

// ------------------------------------------------------------------- config

#define PIN_LED      3            // Timer2 PWM, ~490 Hz, gate of Q1
#define PIN_FAN_PWM  9            // Timer1 OC1A, 25 kHz
#define PIN_FAN_TACH 2            // INT0
#define PIN_HEARTBEAT 13          // onboard LED, no hardware PWM here

const uint32_t DAY_MS    = 86400000UL;
const uint16_t CFG_MAGIC = 0xB452;      // bumped: struct changed since step 1
const int      EE_ADDR   = 0;

struct Config {
  uint16_t magic;
  uint16_t photoperiodMin;   // minutes of light per 24 h
  uint16_t startMin;         // minute of the cycle when the light comes up
  uint8_t  level;            // 0-255 light duty at full brightness
  uint16_t rampSec;          // sunrise / sunset fade length
  uint8_t  fanDuty;          // 0-255 fan duty
  uint16_t logSec;           // status line interval
  uint16_t sum;
};

Config cfg;

void cfgDefaults() {
  cfg.magic          = CFG_MAGIC;
  cfg.photoperiodMin = 18 * 60;   // 18 h on, 6 h dark
  cfg.startMin       = 0;
  cfg.level          = 255;
  cfg.rampSec        = 300;
  cfg.fanDuty        = 100;       // ~40 %, quiet but moving air
  cfg.logSec         = 60;
  cfg.sum            = 0;
}

uint16_t cfgChecksum() {
  const uint8_t *p = (const uint8_t *)&cfg;
  uint16_t s = 0;
  for (size_t i = 0; i < sizeof(Config) - sizeof(cfg.sum); i++) s += p[i] * 31 + 7;
  return s;
}

void cfgSave() {
  cfg.sum = cfgChecksum();
  EEPROM.put(EE_ADDR, cfg);
}

bool cfgLoad() {
  Config disk;
  EEPROM.get(EE_ADDR, disk);
  Config keep = cfg;
  cfg = disk;
  if (disk.magic == CFG_MAGIC && disk.sum == cfgChecksum()) return true;
  cfg = keep;
  return false;
}

// ---------------------------------------------------------------------- fan

// Fast PWM with ICR1 as TOP. 16 MHz / (639 + 1) = 25 kHz.
const uint16_t FAN_TOP = 639;

volatile uint16_t tachPulses   = 0;
uint32_t          tachWindowMs = 0;
uint16_t          fanRPM       = 0;

void tachISR() { tachPulses++; }

void fanInit() {
  pinMode(PIN_FAN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);              // non-inverting on OC1A
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);   // fast PWM, ICR1 top, no prescale
  ICR1   = FAN_TOP;
  OCR1A  = 0;
}

void fanApply() {
  OCR1A = ((uint32_t)FAN_TOP * cfg.fanDuty) / 255;
}

// RPM is measured against real time, never the FAST-scaled schedule clock.
void fanSampleRPM() {
  uint32_t now = millis();
  uint32_t window = now - tachWindowMs;
  if (window < 1000) return;

  noInterrupts();
  uint16_t pulses = tachPulses;
  tachPulses = 0;
  interrupts();
  tachWindowMs = now;

  // two pulses per revolution
  fanRPM = (uint16_t)(((uint32_t)pulses * 60000UL) / (window * 2UL));
}

// -------------------------------------------------------------- timekeeping

enum Mode { MODE_AUTO, MODE_ON, MODE_OFF };
Mode mode = MODE_AUTO;

uint32_t lastTickMs   = 0;
uint32_t cycleMs      = 0;
uint32_t subSecondMs  = 0;
uint32_t uptimeSec    = 0;
uint16_t dayCount     = 0;
uint32_t stateElapsed = 0;
bool     lightIsOn    = false;

uint16_t timeScale = 1;       // FAST n speeds the schedule up for bench testing

void tick() {
  uint32_t now = millis();
  uint32_t raw = now - lastTickMs;        // unsigned, so rollover is harmless
  if (raw == 0) return;
  lastTickMs = now;

  uint32_t delta = raw * timeScale;

  cycleMs += delta;
  while (cycleMs >= DAY_MS) {
    cycleMs -= DAY_MS;
    dayCount++;
  }

  stateElapsed += delta;

  subSecondMs += delta;
  while (subSecondMs >= 1000) {
    subSecondMs -= 1000;
    uptimeSec++;
  }
}

// ---------------------------------------------------------------- heartbeat

// D13 is not a PWM pin on the Uno, so the fade is bit-banged here. Because it
// is driven from loop(), a frozen LED means a frozen sketch — which is the
// whole point of a heartbeat.

bool     hbEnabled   = true;
uint32_t hbPhaseMs   = 0;      // position within the breathe cycle
uint32_t hbLastMs    = 0;
uint32_t hbSlotUs    = 0;      // start of the current software-PWM slot
uint8_t  hbLevel     = 0;

const uint16_t HB_SLOT_US = 4000;   // 250 Hz, fast enough to look steady

void heartbeatUpdate() {
  if (!hbEnabled) return;

  // Breathe faster when the light is under manual override, so it is obvious
  // at a glance that the schedule is not in control.
  uint16_t periodMs = (mode == MODE_AUTO) ? 3000 : 800;

  uint32_t nowMs = millis();
  hbPhaseMs += nowMs - hbLastMs;
  hbLastMs = nowMs;
  while (hbPhaseMs >= periodMs) hbPhaseMs -= periodMs;

  // triangle wave, then squared so the fade looks linear to the eye
  uint32_t half = periodMs / 2;
  uint16_t tri = (hbPhaseMs < half)
                 ? (uint16_t)((hbPhaseMs * 255UL) / half)
                 : (uint16_t)(((periodMs - hbPhaseMs) * 255UL) / half);
  hbLevel = (uint8_t)(((uint32_t)tri * tri) / 255UL);

  uint32_t nowUs = micros();
  if (nowUs - hbSlotUs >= HB_SLOT_US) hbSlotUs = nowUs;
  uint16_t into = (uint16_t)(nowUs - hbSlotUs);
  uint16_t onFor = (uint16_t)(((uint32_t)HB_SLOT_US * hbLevel) / 255UL);
  digitalWrite(PIN_HEARTBEAT, into < onFor ? HIGH : LOW);
}

// delay() with the heartbeat still running, for the blocking parts of SWEEP.
void waitMs(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) heartbeatUpdate();
}

// -------------------------------------------------------------------- light

bool scheduleSaysOn(uint32_t &msLeft) {
  uint32_t onAt = (uint32_t)cfg.startMin * 60000UL;
  uint32_t dur  = (uint32_t)cfg.photoperiodMin * 60000UL;

  if (dur == 0)      { msLeft = DAY_MS; return false; }
  if (dur >= DAY_MS) { msLeft = DAY_MS; return true;  }

  uint32_t rel = (cycleMs >= onAt) ? (cycleMs - onAt) : (cycleMs + DAY_MS - onAt);
  if (rel < dur) { msLeft = dur - rel; return true; }
  msLeft = DAY_MS - rel;
  return false;
}

// Duty is linear in photon output, so no gamma correction here.
uint8_t rampedLevel(uint32_t msLeft) {
  uint32_t rampMs = (uint32_t)cfg.rampSec * 1000UL;
  if (rampMs == 0) return cfg.level;
  if (stateElapsed < rampMs)
    return (uint8_t)(((uint32_t)cfg.level * stateElapsed) / rampMs);
  if (msLeft < rampMs)
    return (uint8_t)(((uint32_t)cfg.level * msLeft) / rampMs);
  return cfg.level;
}

uint8_t currentDuty = 0;

void updateLight() {
  uint32_t msLeft;
  bool sched = scheduleSaysOn(msLeft);

  if (sched != lightIsOn) {
    lightIsOn = sched;
    stateElapsed = 0;
    Serial.print(F("EVT,"));
    Serial.print(uptimeSec);
    Serial.println(sched ? F(",sunrise") : F(",sunset"));
  }

  if      (mode == MODE_ON)  currentDuty = cfg.level;
  else if (mode == MODE_OFF) currentDuty = 0;
  else                       currentDuty = lightIsOn ? rampedLevel(msLeft) : 0;

  analogWrite(PIN_LED, currentDuty);
}

// -------------------------------------------------------------------- output

void printClock(uint32_t sec) {
  uint32_t h = sec / 3600UL;
  uint8_t  m = (sec % 3600UL) / 60UL;
  uint8_t  s = sec % 60UL;
  if (h < 10) Serial.print('0');
  Serial.print(h); Serial.print(':');
  if (m < 10) Serial.print('0');
  Serial.print(m); Serial.print(':');
  if (s < 10) Serial.print('0');
  Serial.print(s);
}

const char *modeName() {
  if (mode == MODE_ON)  return "FORCED-ON";
  if (mode == MODE_OFF) return "FORCED-OFF";
  return "AUTO";
}

void printLog() {
  uint32_t msLeft;
  bool sched = scheduleSaysOn(msLeft);
  Serial.print(F("LOG,"));
  printClock(cycleMs / 1000UL);              Serial.print(',');
  Serial.print(dayCount);                    Serial.print(',');
  Serial.print(sched ? F("ON") : F("OFF"));  Serial.print(',');
  Serial.print(currentDuty);                 Serial.print(',');
  printClock(stateElapsed / 1000UL);         Serial.print(',');
  printClock(msLeft / 1000UL);               Serial.print(',');
  Serial.print(cfg.fanDuty);                 Serial.print(',');
  Serial.print(fanRPM);                      Serial.print(',');
  Serial.println(modeName());
}

void printStatus() {
  uint32_t msLeft;
  bool sched = scheduleSaysOn(msLeft);
  Serial.println(F("--- status ---"));
  Serial.print(F("cycle position  ")); printClock(cycleMs / 1000UL);
  Serial.print(F("   day "));          Serial.println(dayCount);
  Serial.print(F("photoperiod     ")); Serial.print(cfg.photoperiodMin / 60.0f, 1);
  Serial.print(F(" h on, "));          Serial.print((1440 - cfg.photoperiodMin) / 60.0f, 1);
  Serial.println(F(" h dark"));
  Serial.print(F("lights on at    ")); printClock((uint32_t)cfg.startMin * 60UL);
  Serial.println();
  Serial.print(F("schedule says   ")); Serial.println(sched ? F("ON") : F("OFF"));
  Serial.print(F("changes in      ")); printClock(msLeft / 1000UL); Serial.println();
  Serial.print(F("mode            ")); Serial.println(modeName());
  Serial.print(F("light duty      ")); Serial.print(currentDuty);
  Serial.print(F(" / ")); Serial.println(cfg.level);
  Serial.print(F("ramp            ")); Serial.print(cfg.rampSec); Serial.println(F(" s"));
  Serial.print(F("fan duty        ")); Serial.print(cfg.fanDuty);
  Serial.print(F(" / 255  ("));
  Serial.print((cfg.fanDuty * 100) / 255); Serial.println(F(" %)"));
  Serial.print(F("fan speed       ")); Serial.print(fanRPM); Serial.println(F(" rpm"));
  Serial.print(F("clock speed     ")); Serial.print(timeScale); Serial.println(F("x"));
  Serial.println(F("--------------"));
}

void printHelp() {
  Serial.println(F("commands, one per line:"));
  Serial.println(F("  HELP              this text"));
  Serial.println(F("  STATUS            current state"));
  Serial.println(F("  ON / OFF / AUTO   force the light, or return to the schedule"));
  Serial.println(F("  HOURS <h>         photoperiod in hours, eg HOURS 20"));
  Serial.println(F("  MINUTES <m>       photoperiod in minutes"));
  Serial.println(F("  START <hh:mm>     when in the cycle the light comes up"));
  Serial.println(F("  NOW <hh:mm>       set the current cycle position"));
  Serial.println(F("  LEVEL <0-255>     light brightness at full output"));
  Serial.println(F("  RAMP <sec>        sunrise and sunset fade length"));
  Serial.println(F("  FAN <0-255>       fan speed"));
  Serial.println(F("  SWEEP             step the fan through its range, report rpm"));
  Serial.println(F("  HB <0|1>          onboard heartbeat LED off / on"));
  Serial.println(F("  LOG <sec>         status line interval"));
  Serial.println(F("  FAST <n>          run the schedule n times faster"));
  Serial.println(F("  SAVE              store settings in EEPROM"));
  Serial.println(F("  DEFAULTS          reload basil defaults (SAVE to keep them)"));
}

// Steps through the duty range reporting measured rpm, so you can find where
// your particular fan stalls and where it stops speeding up. Blocks for about
// half a minute.
void fanSweep() {
  uint8_t saved = cfg.fanDuty;
  Serial.println(F("duty,percent,rpm"));
  for (uint16_t d = 0; d <= 256; d += 32) {
    cfg.fanDuty = (uint8_t)(d > 255 ? 255 : d);
    fanApply();
    waitMs(3000);                      // let the fan settle at the new duty
    noInterrupts(); tachPulses = 0; interrupts();
    tachWindowMs = millis();
    waitMs(2000);
    fanSampleRPM();
    Serial.print(cfg.fanDuty); Serial.print(',');
    Serial.print((cfg.fanDuty * 100) / 255); Serial.print(',');
    Serial.println(fanRPM);
  }
  cfg.fanDuty = saved;
  fanApply();
  Serial.println(F("OK sweep done, duty restored"));
  lastTickMs = millis();               // we blocked; do not credit that as cycle time
}

// ------------------------------------------------------------- serial input

char    buf[40];
uint8_t buflen = 0;

long parseTime(const char *tok, long fallback) {
  if (!tok) return fallback;
  const char *colon = strchr(tok, ':');
  if (colon) return atol(tok) * 60L + atol(colon + 1);
  return atol(tok);
}

void ack(const __FlashStringHelper *what, long v) {
  Serial.print(F("OK ")); Serial.print(what);
  Serial.print(F(" = ")); Serial.println(v);
}

void handle(char *line) {
  for (char *p = line; *p; p++) *p = toupper(*p);
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg = strtok(NULL, " \t");

  if      (!strcmp(cmd, "HELP"))   printHelp();
  else if (!strcmp(cmd, "STATUS")) printStatus();
  else if (!strcmp(cmd, "ON"))   { mode = MODE_ON;   Serial.println(F("OK forced on")); }
  else if (!strcmp(cmd, "OFF"))  { mode = MODE_OFF;  Serial.println(F("OK forced off")); }
  else if (!strcmp(cmd, "AUTO")) { mode = MODE_AUTO; Serial.println(F("OK back on schedule")); }
  else if (!strcmp(cmd, "HOURS")) {
    long h = arg ? atol(arg) : 20;
    cfg.photoperiodMin = (uint16_t)constrain(h * 60L, 0L, 1440L);
    ack(F("photoperiod (min)"), cfg.photoperiodMin);
  }
  else if (!strcmp(cmd, "MINUTES")) {
    cfg.photoperiodMin = (uint16_t)constrain(arg ? atol(arg) : 1200L, 0L, 1440L);
    ack(F("photoperiod (min)"), cfg.photoperiodMin);
  }
  else if (!strcmp(cmd, "START")) {
    cfg.startMin = (uint16_t)constrain(parseTime(arg, 0), 0L, 1439L);
    ack(F("start (min)"), cfg.startMin);
  }
  else if (!strcmp(cmd, "NOW")) {
    long m = constrain(parseTime(arg, 0), 0L, 1439L);
    cycleMs = (uint32_t)m * 60000UL;
    stateElapsed = 0;
    ack(F("cycle position (min)"), m);
  }
  else if (!strcmp(cmd, "LEVEL")) {
    cfg.level = (uint8_t)constrain(arg ? atol(arg) : 255L, 0L, 255L);
    ack(F("light level"), cfg.level);
  }
  else if (!strcmp(cmd, "RAMP")) {
    cfg.rampSec = (uint16_t)constrain(arg ? atol(arg) : 300L, 0L, 3600L);
    ack(F("ramp (s)"), cfg.rampSec);
  }
  else if (!strcmp(cmd, "FAN")) {
    cfg.fanDuty = (uint8_t)constrain(arg ? atol(arg) : 100L, 0L, 255L);
    fanApply();
    ack(F("fan duty"), cfg.fanDuty);
  }
  else if (!strcmp(cmd, "SWEEP")) fanSweep();
  else if (!strcmp(cmd, "HB")) {
    hbEnabled = arg ? (atol(arg) != 0) : true;
    if (!hbEnabled) digitalWrite(PIN_HEARTBEAT, LOW);
    Serial.println(hbEnabled ? F("OK heartbeat on") : F("OK heartbeat off"));
  }
  else if (!strcmp(cmd, "LOG")) {
    cfg.logSec = (uint16_t)constrain(arg ? atol(arg) : 60L, 1L, 3600L);
    ack(F("log interval (s)"), cfg.logSec);
  }
  else if (!strcmp(cmd, "FAST")) {
    timeScale = (uint16_t)constrain(arg ? atol(arg) : 1L, 1L, 2000L);
    ack(F("clock speed"), timeScale);
  }
  else if (!strcmp(cmd, "SAVE")) { cfgSave(); Serial.println(F("OK saved")); }
  else if (!strcmp(cmd, "DEFAULTS")) {
    cfgDefaults();
    fanApply();
    Serial.println(F("OK basil defaults loaded, SAVE to keep them"));
  }
  else { Serial.print(F("ERR unknown: ")); Serial.println(cmd); }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[buflen] = '\0';
      if (buflen) handle(buf);
      buflen = 0;
    } else if (buflen < sizeof(buf) - 1) {
      buf[buflen++] = c;
    }
  }
}

// --------------------------------------------------------------------- main

uint32_t nextLogMs = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, 0);
  pinMode(PIN_FAN_TACH, INPUT_PULLUP);
  pinMode(PIN_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_HEARTBEAT, LOW);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) { }

  if (!cfgLoad()) {
    cfgDefaults();
    Serial.println(F("# no valid saved settings, using basil defaults"));
  }

  fanInit();
  fanApply();
  attachInterrupt(digitalPinToInterrupt(PIN_FAN_TACH), tachISR, FALLING);

  lastTickMs   = millis();
  tachWindowMs = lastTickMs;
  hbLastMs     = lastTickMs;
  hbSlotUs     = micros();
  nextLogMs    = lastTickMs + 1000;

  Serial.println(F("# grow light + fan ready. HELP for commands."));
  Serial.println(F("# LOG,cycle_time,day,scheduled,light_duty,state_for,until_change,fan_duty,rpm,mode"));
  printStatus();
}

void loop() {
  heartbeatUpdate();          // first, so it keeps running whatever else does
  tick();
  pollSerial();
  updateLight();
  fanSampleRPM();

  uint32_t now = millis();
  if ((int32_t)(now - nextLogMs) >= 0) {
    nextLogMs = now + (uint32_t)cfg.logSec * 1000UL;
    printLog();
  }
}
