/*
 * ============================================================================
 *  ARIA — Wearable Breathing Monitor
 *  Tacconi & Di Staso  ·  Sapienza University of Rome  ·  v8
 * ============================================================================
 *
 *  HARDWARE
 *  --------
 *  - Arduino Uno R4 WiFi (Renesas RA4M1)
 *  - MPU6050 GY-521          : I2C @ 0x68 (AD0->GND)
 *                              Body axis convention: X = forward, Y = right
 *  - MPXV7002DP              : ratiometric ±2 kPa differential pressure
 *                              Vout -> A3, port P1 -> tube -> chest bladder,
 *                              P2 open to atmosphere
 *  - Pancake vibration motor : low-side switched by PN2222 NPN
 *                              D3 (PWM) -> 1 kΩ -> base
 *                              1N4007 flyback diode (band toward +5V)
 *                              100 nF parallel cap across motor
 *  - LED (D13)               : visual state indicator
 *
 *  STATE MACHINE
 *  -------------
 *  Six states, transitions driven by gestures, sensor anomalies, or commands.
 *
 *    OFF               idle, only listens for double pinch (or 'o')
 *    MONITORING        active breath tracking + anomaly detection
 *    RELAX             guided breathing, motor plays a vibration pattern
 *    RECALIBRATING     silent non-blocking auto-cal for a new posture (~20s)
 *    HYPERVENT_RESCUE  90 s descending-rhythm motor pulses (bpm > 24)
 *    APNOEA_ALERT      continuous vibration after 12 s without a breath,
 *                      exits automatically when a breath resumes
 *
 *  RELAX PATTERNS (cycled by single-pinch while in RELAX)
 *  ------------------------------------------------------
 *  Each pattern has its own per-phase haptic cue, so the user feels the
 *  rhythm without looking at the LED or app:
 *
 *    3-3       INHALE ramp-up 180 / EXHALE ramp-down 120          (6 s)
 *    4-7-8     INHALE ramp-up 200 / HOLD pulse 55 / EXHALE 160   (19 s)
 *    Box       INHALE 200 / HOLD pulse 50 / EXHALE 160 / REST 0  (16 s)
 *
 *  Ramps use quadratic ease curves to feel natural (Weber-Fechner).
 *  Pulse phases are 350 ms ON / 650 ms OFF at low intensity (metronome).
 *
 *  CONTROL INTERFACES
 *  ------------------
 *  Serial @ 115200 baud:
 *    o     toggle session (= double pinch)
 *    r     toggle Relax (= long pinch)
 *    c     recalibrate current posture (or initial guided cal if OFF)
 *    s     print full status line
 *    h     force HYPERVENT_RESCUE (demo)
 *    a     force APNOEA_ALERT (demo)
 *    x     wipe Flash profiles, reboot to start fresh
 *    3/4/8 switch Relax pattern (3-3 / 4-7-8 / Box)
 *    ?     list commands
 *
 *  WiFi (Access Point mode, SSID "Aria", pass "Aria123!"):
 *    GET /                    Material 3 + glass dashboard
 *    GET /data                full JSON: state, posture, bpm, regularity,
 *                             dp, baseline, thresholds, counters, profiles,
 *                             active pattern + phase + phase progress
 *    GET /cmd?a=ACTION[&p=N]  command endpoint, mirrors all serial commands
 *
 * ============================================================================
 */

#include <Wire.h>
#include <WiFiS3.h>
#include <EEPROM.h>
#include <math.h>

// ============== HARDWARE ==============
const uint8_t MPU_ADDR     = 0x68;
const int     PIN_PRESSURE = A3;
const int     PIN_MOTOR    = 3;
const int     LED_PIN      = LED_BUILTIN;

// ============== WIFI ==============
const char* AP_SSID = "Aria";
const char* AP_PASS = "Aria123!";
WiFiServer wifiServer(80);

// ============== TIMING ==============
const unsigned long SAMPLE_INTERVAL_MS = 20;
const unsigned long STATUS_PRINT_MS    = 1000;

// ============== THRESHOLDS ==============
const float MOVEMENT_ACTIVITY_MIN = 0.25f;
const int   POSTURE_HYSTERESIS    = 25;
const int   ACTIVITY_WINDOW       = 50;

const float BREATH_THR_MIN  = 0.003f;
const float PINCH_THR_MIN   = 0.10f;
const float BASELINE_ALPHA  = 0.0005f;

const unsigned long PINCH_SHORT_MAX_MS  = 1800;   // was 1500
const unsigned long PINCH_LONG_MIN_MS   = 3000;   // unchanged
const unsigned long DOUBLE_PINCH_GAP    = 1500;   // was 1200

const unsigned long PRE_APNOEA_WARN_MS  = 8000;
const unsigned long APNOEA_TIMEOUT_MS   = 12000;
const float         HYPERVENT_BPM_THR   = 24.0f;
const unsigned long RESCUE_DURATION_MS  = 90000;

// Silent recal phases (auto on posture change, ~20s)
const unsigned long RECAL_WAIT_MS        = 3000;
const unsigned long RECAL_BASELINE_MS    = 5000;
const unsigned long RECAL_BREATH_AMP_MS  = 12000;
const float         RECAL_MOTION_RESTART = 0.4f;

// Initial guided calibration phases (first boot / OFF state, ~38s)
const unsigned long INITCAL_SETTLE_MS    = 3000;
const unsigned long INITCAL_BASELINE_MS  = 5000;
const unsigned long INITCAL_BREATH_MS    = 15000;
const unsigned long INITCAL_PINCH_MS     = 15000;

// ============== STATES ==============
enum State {
  STATE_OFF,
  STATE_MONITORING,
  STATE_RELAX,
  STATE_HYPERVENT_RESCUE,
  STATE_APNOEA_ALERT,
  STATE_RECALIBRATING
};
State currentState     = STATE_OFF;
State stateBeforeRecal = STATE_MONITORING;

enum Posture { POSTURE_UNKNOWN, POSTURE_UPRIGHT, POSTURE_LYING, POSTURE_MOVING };
Posture currentPosture = POSTURE_UNKNOWN;
Posture rawPosture     = POSTURE_UNKNOWN;
int     postureStable  = 0;

// ============== PER-POSTURE PROFILES ==============
struct PostureProfile {
  bool  calibrated;
  float baseline;
  float breathThr;
  float pinchThr;
};
PostureProfile profiles[4];                 // indexed by Posture enum
const uint32_t PROFILE_MAGIC = 0xCAFE2026;  // validates Flash data

// ============== RELAX PATTERN ENGINE ==============
// Each pattern is a sequence of phases. Each phase has a duration and a
// haptic cue type that maps to a motor PWM profile:
//   PHASE_SILENT     motor off (rest)
//   PHASE_STEADY     constant intensity
//   PHASE_RAMP_UP    0 -> peak, quadratic ease (inhale: "breathe in deeper")
//   PHASE_RAMP_DOWN  peak -> 0, quadratic ease (exhale: "let it out")
//   PHASE_PULSE      350 ms ON / 650 ms OFF at low intensity (hold)
enum PhaseType : uint8_t { PHASE_SILENT, PHASE_STEADY, PHASE_RAMP_UP, PHASE_RAMP_DOWN, PHASE_PULSE };

struct RelaxPhase {
  uint16_t  durationMs;
  PhaseType type;
  uint8_t   peak;         // 0..255 PWM peak for this phase
};

struct RelaxPattern {
  const char* name;
  uint8_t     phaseCount;
  RelaxPhase  phases[4];
  uint16_t    cycleMs;            // sum of phase durations
  const char* phaseLabels[4];     // human-readable for dashboard
};

const RelaxPattern PATTERNS[3] = {
  // ---- 3-3: simple inhale/exhale, 6 s cycle ----
  { "3-3", 2,
    { {3000, PHASE_RAMP_UP, 180}, {3000, PHASE_RAMP_DOWN, 120} },
    6000,
    {"INHALE", "EXHALE", "", ""}
  },
  // ---- 4-7-8: inhale, hold, long exhale, 19 s cycle ----
  { "4-7-8", 3,
    { {4000, PHASE_RAMP_UP, 200}, {7000, PHASE_PULSE, 55}, {8000, PHASE_RAMP_DOWN, 160} },
    19000,
    {"INHALE", "HOLD", "EXHALE", ""}
  },
  // ---- Box 4-4-4-4: inhale, hold, exhale, rest, 16 s cycle ----
  { "Box", 4,
    { {4000, PHASE_RAMP_UP, 200}, {4000, PHASE_PULSE, 50}, {4000, PHASE_RAMP_DOWN, 160}, {4000, PHASE_SILENT, 0} },
    16000,
    {"INHALE", "HOLD", "EXHALE", "REST"}
  }
};

int      currentRelaxPattern = 2;   // default: Box
uint8_t  currentRelaxPhase   = 0;   // updated every relaxTick()
uint32_t relaxCycleStart     = 0;   // millis() at start of current cycle
float    currentRelaxProg    = 0;   // 0..1 progress within current phase

// Forward declaration (used by changeState before relaxTick is defined)
void startRelaxCycle();
void relaxTick();
void startSilentRecal();
void startInitialCalibration();
void printSessionSummary();

// Reset relax cycle to phase 0 — call when entering RELAX or switching pattern
void startRelaxCycle() {
  relaxCycleStart   = millis();
  currentRelaxPhase = 0;
  currentRelaxProg  = 0;
}

// Called every loop tick while in STATE_RELAX. Computes which phase we are
// in within the active pattern cycle and drives the motor accordingly.
void relaxTick() {
  if (currentState != STATE_RELAX) { analogWrite(PIN_MOTOR, 0); return; }
  const RelaxPattern& p = PATTERNS[currentRelaxPattern];
  uint32_t t = (millis() - relaxCycleStart) % p.cycleMs;   // ms into cycle
  uint32_t acc = 0;
  for (uint8_t i = 0; i < p.phaseCount; i++) {
    const RelaxPhase& ph = p.phases[i];
    if (t < acc + ph.durationMs) {
      currentRelaxPhase = i;
      float prog = float(t - acc) / float(ph.durationMs);   // 0..1 in phase
      currentRelaxProg = prog;
      uint8_t out = 0;
      switch (ph.type) {
        case PHASE_SILENT:    out = 0; break;
        case PHASE_STEADY:    out = ph.peak; break;
        case PHASE_RAMP_UP:   out = (uint8_t)(ph.peak * prog * prog); break;
        case PHASE_RAMP_DOWN: out = (uint8_t)(ph.peak * (1.0f - prog) * (1.0f - prog)); break;
        case PHASE_PULSE: {
          // 350 ms ON / 650 ms OFF — feels like a calm metronome
          uint32_t cyc = (t - acc) % 1000;
          out = (cyc < 350) ? ph.peak : 0;
          break;
        }
      }
      analogWrite(PIN_MOTOR, out);
      return;
    }
    acc += ph.durationMs;
  }
}

// ============== FLASH PROFILES ==============
void loadProfilesFromFlash() {
  uint32_t magic;
  int addr = 0;
  EEPROM.get(addr, magic);
  if (magic != PROFILE_MAGIC) {
    Serial.println(F("[FLASH] No saved profiles, starting fresh"));
    for (int i = 0; i < 4; i++) profiles[i].calibrated = false;
    return;
  }
  addr += sizeof(magic);
  for (int i = 0; i < 4; i++) {
    EEPROM.get(addr, profiles[i]);
    addr += sizeof(PostureProfile);
  }
  Serial.println(F("[FLASH] Profiles loaded:"));
  for (int i = 1; i <= 3; i++) {
    Serial.print(F("  posture "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(profiles[i].calibrated ? F("CALIBRATED") : F("missing"));
  }
}

void saveProfilesToFlash() {
  int addr = 0;
  EEPROM.put(addr, PROFILE_MAGIC);
  addr += sizeof(PROFILE_MAGIC);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(addr, profiles[i]);
    addr += sizeof(PostureProfile);
  }
  Serial.println(F("[FLASH] Profiles saved"));
}

// ============== SENSOR DATA ==============
float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float pressure_v = 0, dp_kpa = 0, dp_baseline = 0;
float magBuf[ACTIVITY_WINDOW];
int   magBufIdx  = 0;
bool  magBufFull = false;

// ============== BREATH / PINCH ==============
float breathThreshold = 0.05f;
float pinchThreshold  = 0.30f;
float currentBreathPeak = 0;
bool  inBreath = false;
unsigned long breathStartTime = 0;
unsigned long lastBreathTime  = 0;
bool  hasDetectedBreath = false;

float currentBpm = 0;
const int BREATH_HIST_SIZE = 10;
unsigned long breathIntervals[BREATH_HIST_SIZE];
int breathHistIdx = 0, breathHistCount = 0;

bool inPinch = false;
unsigned long pinchStartTime  = 0;
unsigned long lastReleaseTime = 0;
int  pendingPinchCount = 0;

bool preApnoeaWarned = false;

unsigned long rescueStartTime = 0;
float rescueStartBpm = 0;
unsigned long lastRescuePulse = 0;

// ============== RECAL STATE ==============
enum RecalPhase {
  RECAL_NONE,
  // Silent recal (auto-triggered, no user action required beyond stillness)
  RECAL_WAIT, RECAL_BASELINE, RECAL_BREATH_AMP,
  // Initial guided cal (first run, includes active pinch capture)
  INITCAL_SETTLE, INITCAL_BASELINE, INITCAL_BREATH, INITCAL_PINCH
};
RecalPhase recalPhase = RECAL_NONE;
unsigned long recalPhaseStart = 0;
float recalBaselineSum   = 0;
int   recalBaselineCount = 0;
float recalBreathPeak    = 0;
float recalPinchPeak     = 0;

// ============== SESSION STATS ==============
unsigned long sessionStartTime = 0;
unsigned long totalBreaths = 0;
float sumBpm = 0;
int   bpmSamples = 0;
int   apnoeaEvents = 0, hyperventEvents = 0;
unsigned long timeInState[6]   = {0,0,0,0,0,0};
unsigned long timeInPosture[4] = {0,0,0,0};
unsigned long lastStateChange   = 0;
unsigned long lastPostureChange = 0;

// ============== TIMING ==============
unsigned long lastSample      = 0;
unsigned long lastStatusPrint = 0;
unsigned long ledPhaseStart   = 0;
bool ledState = false;

// ============== SENSOR READS ==============
void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  ax_g = raw[0]/16384.0f; ay_g = raw[1]/16384.0f; az_g = raw[2]/16384.0f;
  gx_dps = raw[4]/131.0f; gy_dps = raw[5]/131.0f; gz_dps = raw[6]/131.0f;
}

void readPressure() {
  int adc = analogRead(PIN_PRESSURE);
  pressure_v = (adc * 5.0f) / 1023.0f;
  dp_kpa = pressure_v - 2.5f;
}

// ============== POSTURE ==============
float computeActivity() {
  int n = magBufFull ? ACTIVITY_WINDOW : magBufIdx;
  if (n < 10) return 0;
  float mn = 1000, mx = -1000;
  for (int i = 0; i < n; i++) {
    if (magBuf[i] < mn) mn = magBuf[i];
    if (magBuf[i] > mx) mx = magBuf[i];
  }
  return mx - mn;
}

void updateActivityBuffer() {
  float mag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  magBuf[magBufIdx] = mag;
  magBufIdx = (magBufIdx + 1) % ACTIVITY_WINDOW;
  if (magBufIdx == 0) magBufFull = true;
}

Posture classifyPosture() {
  float absX = fabs(ax_g), absY = fabs(ay_g), absZ = fabs(az_g);
  float activity = computeActivity();
  if (absZ >= absX && absZ >= absY) {
    if (activity > MOVEMENT_ACTIVITY_MIN) return POSTURE_MOVING;
    return POSTURE_UPRIGHT;
  }
  return POSTURE_LYING;
}

const char* postureToString(Posture p) {
  switch (p) {
    case POSTURE_UPRIGHT: return "ERETTO";
    case POSTURE_LYING:   return "SDRAIATO";
    case POSTURE_MOVING:  return "IN_MOVIMENTO";
    default:              return "?";
  }
}

void onPostureChange(Posture oldP, Posture newP) {
  if (newP == POSTURE_MOVING) {
    Serial.println(F("[POSTURE] In movement, detection suspended"));
    return;
  }
  if (profiles[newP].calibrated) {
    dp_baseline     = profiles[newP].baseline;
    breathThreshold = profiles[newP].breathThr;
    pinchThreshold  = profiles[newP].pinchThr;
    Serial.print(F("[PROFILE] Restored for "));
    Serial.println(postureToString(newP));
    return;
  }
  if (currentState == STATE_MONITORING) {
    Serial.print(F("[PROFILE] No profile for "));
    Serial.print(postureToString(newP));
    Serial.println(F(", starting silent recalibration"));
    startSilentRecal();
  } else {
    Serial.print(F("[PROFILE] No profile for "));
    Serial.print(postureToString(newP));
    Serial.println(F(", will recalibrate when monitoring resumes"));
  }
}

void updatePosture() {
  Posture c = classifyPosture();
  if (c == rawPosture) {
    // Same as candidate — accumulate evidence (capped to prevent overflow)
    if (postureStable < POSTURE_HYSTERESIS + 20) postureStable++;
    if (postureStable >= POSTURE_HYSTERESIS && c != currentPosture) {
      Posture old = currentPosture;
      timeInPosture[currentPosture] += millis() - lastPostureChange;
      lastPostureChange = millis();
      currentPosture = c;
      Serial.print(F("[POSTURE] ")); Serial.print(postureToString(old));
      Serial.print(F(" -> ")); Serial.println(postureToString(c));
      onPostureChange(old, c);
    }
  } else if (c == currentPosture) {
    // Brief jitter back to currently-committed posture — undo evidence for candidate
    // rather than fully resetting it. Lets the user commit to a new posture even with
    // occasional flickers across the activity boundary.
    if (postureStable > 0) postureStable--;
    if (postureStable == 0) rawPosture = currentPosture;
  } else {
    // A different posture appeared — treat as new candidate
    rawPosture = c;
    postureStable = 1;
  }
}

// ============== STATE MACHINE ==============
const char* stateToString(State s) {
  switch (s) {
    case STATE_OFF:              return "OFF";
    case STATE_MONITORING:       return "MONITORING";
    case STATE_RELAX:            return "RELAX";
    case STATE_HYPERVENT_RESCUE: return "HYPERVENT_RESCUE";
    case STATE_APNOEA_ALERT:     return "APNOEA_ALERT";
    case STATE_RECALIBRATING:    return "RECALIBRATING";
  }
  return "?";
}

void changeState(State s) {
  if (s == currentState) return;
  timeInState[currentState] += millis() - lastStateChange;
  lastStateChange = millis();
  Serial.print(F("[STATE] "));
  Serial.print(stateToString(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateToString(s));
  currentState = s;
  switch (s) {
    case STATE_OFF:
      analogWrite(PIN_MOTOR, 0);
      break;
    case STATE_MONITORING:
      analogWrite(PIN_MOTOR, 0);    // silence any leftover relax/alert vibration
      lastBreathTime  = millis();   // reset apnea timer
      preApnoeaWarned = false;
      break;
    case STATE_RELAX:
      startRelaxCycle();             // pattern cycle starts at phase 0
      break;
    case STATE_HYPERVENT_RESCUE:
      rescueStartTime = millis();
      rescueStartBpm  = currentBpm > 8 ? currentBpm : 30;
      lastRescuePulse = millis();
      hyperventEvents++;
      break;
    case STATE_APNOEA_ALERT:
      apnoeaEvents++;
      analogWrite(PIN_MOTOR, 200);
      break;
    case STATE_RECALIBRATING:
      analogWrite(PIN_MOTOR, 0);
      break;
    default: break;
  }
  preApnoeaWarned = false;
}

// ============== SILENT RECAL ==============
void startSilentRecal() {
  if (currentState != STATE_MONITORING && currentState != STATE_RECALIBRATING) return;
  if (currentPosture == POSTURE_MOVING) return;
  stateBeforeRecal = (currentState == STATE_RECALIBRATING) ? STATE_MONITORING : currentState;
  if (currentState != STATE_RECALIBRATING) changeState(STATE_RECALIBRATING);
  recalPhase = RECAL_WAIT;
  recalPhaseStart    = millis();
  recalBaselineSum   = 0;
  recalBaselineCount = 0;
  recalBreathPeak    = 0;
  Serial.print(F("[RECAL] Begin for "));
  Serial.println(postureToString(currentPosture));
}

void abortRecal() {
  recalPhase = RECAL_NONE;
  if (currentState == STATE_RECALIBRATING) changeState(stateBeforeRecal);
}

void runRecalStep() {
  unsigned long now     = millis();
  unsigned long elapsed = now - recalPhaseStart;
  if (currentPosture == POSTURE_MOVING) {
    Serial.println(F("[RECAL] Aborted (movement)"));
    abortRecal();
    return;
  }
  // Motion contamination restarts the phase — except during INITCAL_PINCH,
  // where the user is actively manipulating the tube on purpose.
  if (recalPhase != INITCAL_PINCH && computeActivity() > RECAL_MOTION_RESTART) {
    recalPhaseStart    = now;
    recalBaselineSum   = 0;
    recalBaselineCount = 0;
    recalBreathPeak    = 0;
    recalPinchPeak     = 0;
    return;
  }
  switch (recalPhase) {
    // ---------- Silent recal (3 phases, ~20s) ----------
    case RECAL_WAIT:
      if (elapsed >= RECAL_WAIT_MS) {
        recalPhase = RECAL_BASELINE;
        recalPhaseStart = now;
        Serial.println(F("[RECAL] Phase 2: baseline"));
      }
      break;
    case RECAL_BASELINE:
      recalBaselineSum += dp_kpa;
      recalBaselineCount++;
      if (elapsed >= RECAL_BASELINE_MS) {
        if (recalBaselineCount > 0) dp_baseline = recalBaselineSum / recalBaselineCount;
        recalPhase = RECAL_BREATH_AMP;
        recalPhaseStart = now;
        Serial.println(F("[RECAL] Phase 3: breath amplitude"));
      }
      break;
    case RECAL_BREATH_AMP: {
      float d = fabs(dp_kpa - dp_baseline);
      if (d > recalBreathPeak) recalBreathPeak = d;
      if (elapsed >= RECAL_BREATH_AMP_MS) {
        float bt = recalBreathPeak * 0.25f;
        if (bt < BREATH_THR_MIN) bt = BREATH_THR_MIN;
        float pt = recalBreathPeak * 2.5f;
        if (pt < PINCH_THR_MIN) pt = PINCH_THR_MIN;
        breathThreshold = bt;
        pinchThreshold  = pt;
        profiles[currentPosture].calibrated = true;
        profiles[currentPosture].baseline   = dp_baseline;
        profiles[currentPosture].breathThr  = bt;
        profiles[currentPosture].pinchThr   = pt;
        saveProfilesToFlash();
        Serial.print(F("[RECAL] Complete: baseline=")); Serial.print(dp_baseline, 3);
        Serial.print(F(" breath="));                    Serial.print(bt, 3);
        Serial.print(F(" pinch="));                     Serial.println(pt, 3);
        recalPhase = RECAL_NONE;
        changeState(stateBeforeRecal);
      }
      break;
    }
    // ---------- Initial guided cal (4 phases, ~38s) ----------
    case INITCAL_SETTLE:
      if (elapsed >= INITCAL_SETTLE_MS) {
        recalPhase = INITCAL_BASELINE;
        recalPhaseStart = now;
        Serial.println(F("[INITCAL] Phase 2/4: baseline"));
      }
      break;
    case INITCAL_BASELINE:
      recalBaselineSum += dp_kpa;
      recalBaselineCount++;
      if (elapsed >= INITCAL_BASELINE_MS) {
        if (recalBaselineCount > 0) dp_baseline = recalBaselineSum / recalBaselineCount;
        Serial.print(F("[INITCAL] Baseline set: ")); Serial.println(dp_baseline, 3);
        recalPhase = INITCAL_BREATH;
        recalPhaseStart = now;
        Serial.println(F("[INITCAL] Phase 3/4: breathing (15s)"));
      }
      break;
    case INITCAL_BREATH: {
      float dev = dp_kpa - dp_baseline;
      if (dev > 0 && dev > recalBreathPeak) recalBreathPeak = dev;
      if (elapsed >= INITCAL_BREATH_MS) {
        float bt = recalBreathPeak * 0.4f;
        if (bt < BREATH_THR_MIN) bt = BREATH_THR_MIN;
        breathThreshold = bt;
        Serial.print(F("[INITCAL] Breath thr set: ")); Serial.println(bt, 3);
        recalPhase = INITCAL_PINCH;
        recalPhaseStart = now;
        recalPinchPeak  = 0;
        Serial.println(F("[INITCAL] Phase 4/4: pinch the tube 3-5 times (15s)"));
      }
      break;
    }
    case INITCAL_PINCH: {
      float dev = dp_kpa - dp_baseline;
      if (dev < 0 && -dev > recalPinchPeak) recalPinchPeak = -dev;
      if (elapsed >= INITCAL_PINCH_MS) {
        float pt = recalPinchPeak * 0.25f;
        // Sanity floor: must clearly exceed breath amplitude
        if (pt < recalBreathPeak * 2.5f) pt = recalBreathPeak * 2.5f;
        if (pt < PINCH_THR_MIN)          pt = PINCH_THR_MIN;
        pinchThreshold = pt;
        profiles[currentPosture].calibrated = true;
        profiles[currentPosture].baseline   = dp_baseline;
        profiles[currentPosture].breathThr  = breathThreshold;
        profiles[currentPosture].pinchThr   = pt;
        saveProfilesToFlash();
        Serial.print(F("[INITCAL] Complete: baseline=")); Serial.print(dp_baseline, 3);
        Serial.print(F(" breath=")); Serial.print(breathThreshold, 3);
        Serial.print(F(" pinch="));  Serial.println(pt, 3);
        recalPhase = RECAL_NONE;
        changeState(stateBeforeRecal);
        buzz(150, 250);  // completion haptic
      }
      break;
    }
    default: break;
  }
}

// Begin a full 4-phase guided calibration (~38s). Used at boot when no
// profile exists for the current posture, and when the user explicitly
// presses Recalibrate from STATE_OFF.
void startInitialCalibration() {
  if (currentPosture == POSTURE_MOVING) {
    Serial.println(F("[INITCAL] Aborted (movement) — try again when still"));
    return;
  }
  stateBeforeRecal = STATE_OFF;
  if (currentState != STATE_RECALIBRATING) changeState(STATE_RECALIBRATING);
  recalPhase         = INITCAL_SETTLE;
  recalPhaseStart    = millis();
  recalBaselineSum   = 0;
  recalBaselineCount = 0;
  recalBreathPeak    = 0;
  recalPinchPeak     = 0;
  Serial.print(F("[INITCAL] Phase 1/4: settle, for posture "));
  Serial.println(postureToString(currentPosture));
}

// ============== BREATH DETECTION ==============
void detectBreath() {
  float dev = dp_kpa - dp_baseline;
  if (!inBreath && dev > breathThreshold) {        // positive deviation = inhale
    inBreath = true;
    breathStartTime   = millis();
    currentBreathPeak = dev;
  } else if (inBreath) {
    if (dev > currentBreathPeak) currentBreathPeak = dev;
    if (dev < breathThreshold * 0.3f && millis() - breathStartTime > 300) {
      inBreath = false;
      unsigned long now = millis();
      if (hasDetectedBreath) {
        unsigned long interval = now - lastBreathTime;
        if (interval > 1000 && interval < 15000) {
          breathIntervals[breathHistIdx] = interval;
          breathHistIdx = (breathHistIdx + 1) % BREATH_HIST_SIZE;
          if (breathHistCount < BREATH_HIST_SIZE) breathHistCount++;
          unsigned long sum = 0;
          for (int i = 0; i < breathHistCount; i++) sum += breathIntervals[i];
          currentBpm = 60000.0f / ((float)sum / breathHistCount);
          sumBpm += currentBpm;
          bpmSamples++;
        }
      }
      lastBreathTime = now;
      hasDetectedBreath = true;
      totalBreaths++;
      preApnoeaWarned = false;
    }
  }
}

float computeRegularity() {
  if (breathHistCount < 3) return 0;
  float sum = 0;
  for (int i = 0; i < breathHistCount; i++) sum += breathIntervals[i];
  float mean = sum / breathHistCount;
  float sumSq = 0;
  for (int i = 0; i < breathHistCount; i++) {
    float d = breathIntervals[i] - mean;
    sumSq += d * d;
  }
  float stddev = sqrt(sumSq / breathHistCount);
  float cv = stddev / mean;
  float reg = 100.0f - cv * 200.0f;
  if (reg < 0)   reg = 0;
  if (reg > 100) reg = 100;
  return reg;
}

// ============== PINCH ==============
void buzz(int intensity, int durationMs) {
  analogWrite(PIN_MOTOR, intensity);
  delay(durationMs);
  analogWrite(PIN_MOTOR, 0);
}

void handleSinglePinch() {
  Serial.println(F("[PINCH] SINGLE"));
  if (currentState == STATE_RELAX) {
    currentRelaxPattern = (currentRelaxPattern + 1) % 3;
    Serial.print(F("[RELAX] Pattern -> "));
    Serial.println(PATTERNS[currentRelaxPattern].name);
    startRelaxCycle();
    buzz(80, 100);
  }
}

void handleDoublePinch() {
  Serial.println(F("[PINCH] DOUBLE"));
  if (currentState == STATE_RECALIBRATING) return;
  if (currentState == STATE_OFF) {
    // ----- START a fresh session -----
    changeState(STATE_MONITORING);
    buzz(250, 250);
    sessionStartTime = millis();
    totalBreaths = 0;
    sumBpm = 0;
    bpmSamples = 0;
    apnoeaEvents = 0;
    hyperventEvents = 0;
    for (int i = 0; i < 6; i++) timeInState[i]   = 0;
    for (int i = 0; i < 4; i++) timeInPosture[i] = 0;
    lastStateChange   = millis();
    lastPostureChange = millis();
    if (currentPosture != POSTURE_MOVING && !profiles[currentPosture].calibrated) {
      startSilentRecal();
    }
  } else if (currentState == STATE_MONITORING) {
    // ----- STOP session entirely (go to OFF) -----
    timeInState[currentState]       += millis() - lastStateChange;
    timeInPosture[currentPosture]   += millis() - lastPostureChange;
    printSessionSummary();
    lastStateChange   = millis();
    lastPostureChange = millis();
    analogWrite(PIN_MOTOR, 0);
    changeState(STATE_OFF);
    buzz(250, 250);
  } else {
    // ----- RELAX / HYPERVENT_RESCUE / APNOEA_ALERT: resume monitoring -----
    // These are transient sub-states of an active session — dismiss them and
    // return to MONITORING rather than ending the session.
    analogWrite(PIN_MOTOR, 0);
    changeState(STATE_MONITORING);
    buzz(150, 200);
  }
}

void handleLongPinch() {
  Serial.println(F("[PINCH] LONG"));
  if (currentState == STATE_MONITORING) { changeState(STATE_RELAX);      buzz(150, 400); }
  else if (currentState == STATE_RELAX) { changeState(STATE_MONITORING); buzz(150, 400); }
  else if (currentState == STATE_OFF)   { handleDoublePinch();           } // long pinch in OFF = start session
}

void detectPinch() {
  if (currentState == STATE_RECALIBRATING) return;
  float dev = dp_kpa - dp_baseline;
  bool pressed = (dev < -pinchThreshold);          // sign-aware: pinch is NEGATIVE deviation

  if (!inPinch && pressed) {
    inPinch = true;
    pinchStartTime = millis();
    Serial.print(F("[PINCH-START] dev="));
    Serial.print(dev, 3);
    Serial.print(F(" thr=-"));
    Serial.println(pinchThreshold, 3);
  } else if (inPinch) {
    if (!pressed || millis() - pinchStartTime > 10000) {
      unsigned long duration = millis() - pinchStartTime;
      inPinch = false;
      lastReleaseTime = millis();
      Serial.print(F("[PINCH-END] duration="));
      Serial.print(duration);
      Serial.print(F("ms "));
      if (duration >= PINCH_LONG_MIN_MS) {
        Serial.println(F("-> LONG"));
        handleLongPinch();
        pendingPinchCount = 0;
      } else if (duration <= PINCH_SHORT_MAX_MS) {
        pendingPinchCount++;
        Serial.print(F("-> SHORT (pending="));
        Serial.print(pendingPinchCount);
        Serial.println(F(")"));
      } else {
        Serial.print(F("-> IGNORED (dead zone "));
        Serial.print(PINCH_SHORT_MAX_MS);
        Serial.print(F("-"));
        Serial.print(PINCH_LONG_MIN_MS);
        Serial.println(F(")"));
      }
    }
  }
  if (pendingPinchCount > 0 && millis() - lastReleaseTime > DOUBLE_PINCH_GAP) {
    Serial.print(F("[PINCH-RESOLVE] count="));
    Serial.println(pendingPinchCount);
    if (pendingPinchCount == 1)      handleSinglePinch();
    else if (pendingPinchCount >= 2) handleDoublePinch();
    pendingPinchCount = 0;
  }
}

// ============== ANOMALY HANDLERS ==============
void runHyperventRescue() {
  unsigned long elapsed = millis() - rescueStartTime;
  if (elapsed >= RESCUE_DURATION_MS) { changeState(STATE_MONITORING); return; }
  float progress  = (float)elapsed / RESCUE_DURATION_MS;
  float targetBpm = rescueStartBpm - (rescueStartBpm - 8.0f) * progress;
  unsigned long period = (unsigned long)(60000.0f / targetBpm);
  if (millis() - lastRescuePulse >= period) {
    lastRescuePulse = millis();
    buzz(130, 200);
  }
}

void checkApnoea() {
  if (!hasDetectedBreath) return;
  if (currentState != STATE_MONITORING) return;
  if (currentPosture == POSTURE_MOVING) return;      // gated during movement
  unsigned long since = millis() - lastBreathTime;
  if (since >= APNOEA_TIMEOUT_MS) {
    changeState(STATE_APNOEA_ALERT);
  } else if (since >= PRE_APNOEA_WARN_MS && !preApnoeaWarned) {
    preApnoeaWarned = true;
    Serial.println(F("[PRE-APNOEA] Gentle warning"));
    buzz(100, 300);
  }
}

void checkApnoeaResume() {
  if (currentState != STATE_APNOEA_ALERT) return;
  if (millis() - lastBreathTime < 2000 && hasDetectedBreath) {
    analogWrite(PIN_MOTOR, 0);
    changeState(STATE_MONITORING);
  }
}

void checkHyperventilation() {
  if (currentState != STATE_MONITORING) return;
  if (currentPosture == POSTURE_MOVING) return;      // gated during movement
  if (currentBpm > HYPERVENT_BPM_THR && breathHistCount >= 5) {
    changeState(STATE_HYPERVENT_RESCUE);
  }
}

// ============== LED ==============
void updateLed() {
  unsigned long now = millis();
  switch (currentState) {
    case STATE_OFF:        digitalWrite(LED_PIN, LOW);  break;
    case STATE_MONITORING: digitalWrite(LED_PIN, HIGH); break;
    case STATE_RELAX:
      if (now - ledPhaseStart > 1500) { ledState = !ledState; ledPhaseStart = now; }
      digitalWrite(LED_PIN, ledState);
      break;
    case STATE_RECALIBRATING:
      if (now - ledPhaseStart > 300) { ledState = !ledState; ledPhaseStart = now; }
      digitalWrite(LED_PIN, ledState);
      break;
    case STATE_HYPERVENT_RESCUE:
      if (now - ledPhaseStart > 200) { ledState = !ledState; ledPhaseStart = now; }
      digitalWrite(LED_PIN, ledState);
      break;
    case STATE_APNOEA_ALERT:
      if (now - ledPhaseStart > 100) { ledState = !ledState; ledPhaseStart = now; }
      digitalWrite(LED_PIN, ledState);
      break;
  }
}

// ============== STATUS PRINT ==============
void printStatus() {
  Serial.print(F("[STATUS] state="));   Serial.print(stateToString(currentState));
  Serial.print(F(" posture="));         Serial.print(postureToString(currentPosture));
  Serial.print(F(" ax="));   Serial.print(ax_g, 2);
  Serial.print(F(" ay="));   Serial.print(ay_g, 2);
  Serial.print(F(" az="));   Serial.print(az_g, 2);
  Serial.print(F(" dp="));   Serial.print(dp_kpa - dp_baseline, 3);
  Serial.print(F(" bThr=")); Serial.print(breathThreshold, 3);
  Serial.print(F(" pThr=")); Serial.print(pinchThreshold, 3);
  Serial.print(F(" bpm="));  Serial.print(currentBpm, 1);
  Serial.print(F(" reg="));  Serial.print(computeRegularity(), 0);
  // Posture diagnostics — shows whether classifier is stuck or oscillating
  Serial.print(F("% act=")); Serial.print(computeActivity(), 2);
  Serial.print(F(" raw=")); Serial.print(postureToString(classifyPosture()));
  Serial.print(F(" stab=")); Serial.print(postureStable);
  Serial.print(F(" pattern=")); Serial.print(PATTERNS[currentRelaxPattern].name);
  if (currentState == STATE_RELAX) {
    Serial.print(F(" phase="));
    Serial.print(PATTERNS[currentRelaxPattern].phaseLabels[currentRelaxPhase]);
  }
  if (currentState == STATE_RECALIBRATING) {
    Serial.print(F(" recal_phase="));
    switch (recalPhase) {
      case RECAL_WAIT:       Serial.print(F("WAIT"));       break;
      case RECAL_BASELINE:   Serial.print(F("BASELINE"));   break;
      case RECAL_BREATH_AMP: Serial.print(F("BREATH_AMP")); break;
      default:               Serial.print(F("?"));
    }
  }
  Serial.println();
}

void printSessionSummary() {
  unsigned long len = millis() - sessionStartTime;
  float avg = (bpmSamples > 0) ? (sumBpm / bpmSamples) : 0;
  Serial.println();
  Serial.println(F("==================================="));
  Serial.println(F("       SESSION SUMMARY"));
  Serial.println(F("==================================="));
  Serial.print(F("Duration: "));       Serial.print(len / 1000); Serial.println(F(" s"));
  Serial.print(F("Total breaths: "));  Serial.println(totalBreaths);
  Serial.print(F("Average bpm: "));    Serial.println(avg, 1);
  Serial.print(F("Final regularity: ")); Serial.print(computeRegularity(), 0); Serial.println(F("%"));
  Serial.print(F("Apnoea events: "));         Serial.println(apnoeaEvents);
  Serial.print(F("Hyperventilation events: ")); Serial.println(hyperventEvents);
  Serial.println(F("Time per posture:"));
  Serial.print(F("  ERETTO:       ")); Serial.print(timeInPosture[POSTURE_UPRIGHT]/1000); Serial.println(F(" s"));
  Serial.print(F("  SDRAIATO:     ")); Serial.print(timeInPosture[POSTURE_LYING]/1000);   Serial.println(F(" s"));
  Serial.print(F("  IN_MOVIMENTO: ")); Serial.print(timeInPosture[POSTURE_MOVING]/1000);  Serial.println(F(" s"));
  Serial.println(F("==================================="));
  Serial.println();
}

// ============== SERIAL COMMANDS ==============
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'c':
      if (currentState == STATE_MONITORING) startSilentRecal();
      else if (currentState == STATE_OFF)   startInitialCalibration();
      break;
    case 's': printStatus(); break;
    case 'h': if (currentState == STATE_MONITORING) changeState(STATE_HYPERVENT_RESCUE); break;
    case 'a': if (currentState == STATE_MONITORING) changeState(STATE_APNOEA_ALERT);    break;
    case 'r':
      if (currentState == STATE_MONITORING)  changeState(STATE_RELAX);
      else if (currentState == STATE_RELAX)  changeState(STATE_MONITORING);
      break;
    case 'o': handleDoublePinch(); break;
    case 'x':
      if (currentState == STATE_RECALIBRATING) {
        Serial.println(F("[WIPE] Ignored — calibration in progress"));
        break;
      }
      EEPROM.put(0, (uint32_t)0xDEADBEEF);
      for (int i = 0; i < 4; i++) profiles[i].calibrated = false;
      Serial.println(F("[FLASH] Profiles wiped (RAM + Flash)"));
      break;
    case '3':
      currentRelaxPattern = 0;
      if (currentState == STATE_RELAX) startRelaxCycle();
      Serial.println(F("[RELAX] -> 3-3"));
      break;
    case '4':
      currentRelaxPattern = 1;
      if (currentState == STATE_RELAX) startRelaxCycle();
      Serial.println(F("[RELAX] -> 4-7-8"));
      break;
    case '8':
      currentRelaxPattern = 2;
      if (currentState == STATE_RELAX) startRelaxCycle();
      Serial.println(F("[RELAX] -> Box"));
      break;
    case '?':
      Serial.println(F("c=recal s=status o=on/off r=relax h=rescue a=apnoea x=wipe profiles"));
      Serial.println(F("3/4/8=relax pattern  ?=help"));
      break;
  }
}

// ============== WIFI DASHBOARD ==============
// Production page: fetches /data every 400ms, POSTs commands to /cmd.
// Glass design (frosted backdrop blur) + Material 3 tokens.
const char HTML_PAGE[] PROGMEM = R"PAGE(<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aria</title>
<style>
:root{
  --primary:#d0bcff;--on-primary:#381e72;
  --primary-c:#4f378b;--on-primary-c:#eaddff;
  --secondary-c:#4a4458;--on-secondary-c:#e8def8;
  --error:#f2b8b5;--on-error:#601410;
  --on-surface:#e6e0e9;--on-variant:#cac4d0;
  --glass:rgba(38,36,42,.55);
  --glass-hi:rgba(255,255,255,.07);
  --glass-lo:rgba(0,0,0,.25);
  --r-s:14px;--r-m:20px;--r-l:32px;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0;min-height:100vh}
body{
  font-family:'Roboto','SF Pro Display',-apple-system,system-ui,sans-serif;
  color:var(--on-surface);background:#0a0710;
  overflow-x:hidden;padding-bottom:32px;position:relative;
}
body::before,body::after,.bg{
  content:'';position:fixed;border-radius:50%;filter:blur(80px);
  pointer-events:none;z-index:-1;will-change:transform;
}
body::before{width:60vw;height:60vw;background:radial-gradient(circle,#7c4dff 0%,transparent 65%);top:-15vw;left:-15vw;animation:drift1 28s ease-in-out infinite}
body::after{width:55vw;height:55vw;background:radial-gradient(circle,#d0bcff 0%,transparent 60%);bottom:-20vw;right:-20vw;animation:drift2 32s ease-in-out infinite;opacity:.7}
.bg{width:45vw;height:45vw;background:radial-gradient(circle,#ff6b9d 0%,transparent 60%);top:30vh;left:50%;animation:drift3 36s ease-in-out infinite;opacity:.35}
@keyframes drift1{0%,100%{transform:translate(0,0) scale(1)}50%{transform:translate(20vw,15vh) scale(1.2)}}
@keyframes drift2{0%,100%{transform:translate(0,0) scale(1)}50%{transform:translate(-15vw,-10vh) scale(.9)}}
@keyframes drift3{0%,100%{transform:translate(-50%,0)}50%{transform:translate(-30%,-15vh) scale(1.15)}}
.bar{padding:18px 22px;display:flex;align-items:center;gap:14px;position:sticky;top:0;z-index:10;
  background:rgba(15,12,22,.4);backdrop-filter:blur(20px) saturate(160%);-webkit-backdrop-filter:blur(20px) saturate(160%);
  border-bottom:1px solid var(--glass-hi)}
.bar h1{margin:0;font-size:1.35rem;font-weight:400;letter-spacing:.5px;
  background:linear-gradient(135deg,#fff 0%,#d0bcff 100%);-webkit-background-clip:text;background-clip:text;color:transparent}
.pill{margin-left:auto;padding:7px 16px;border-radius:999px;font-size:.7rem;font-weight:600;letter-spacing:.7px;
  background:var(--glass);border:1px solid var(--glass-hi);
  backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);
  color:var(--on-variant);transition:all .3s}
.pill.s-MONITORING{background:rgba(123,200,70,.18);border-color:rgba(181,232,139,.3);color:#c4f59a}
.pill.s-RELAX{background:rgba(80,130,220,.18);border-color:rgba(170,200,255,.3);color:#c4d8ff}
.pill.s-RECALIBRATING{background:rgba(220,160,40,.18);border-color:rgba(245,196,110,.3);color:#ffd699}
.pill.s-HYPERVENT_RESCUE{background:rgba(220,100,40,.2);border-color:rgba(255,176,132,.35);color:#ffc4a0}
.pill.s-APNOEA_ALERT{background:rgba(220,40,40,.22);border-color:rgba(255,175,163,.4);color:#ffb8ad;animation:pulseGlow 1.2s ease-in-out infinite}
@keyframes pulseGlow{0%,100%{box-shadow:0 0 0 0 rgba(255,80,80,.4)}50%{box-shadow:0 0 24px 4px rgba(255,80,80,.3)}}
.wrap{padding:18px;max-width:680px;margin:0 auto;display:grid;gap:16px}
.card{background:var(--glass);backdrop-filter:blur(28px) saturate(180%);-webkit-backdrop-filter:blur(28px) saturate(180%);
  border-radius:var(--r-l);padding:22px;border:1px solid var(--glass-hi);
  box-shadow:0 12px 40px var(--glass-lo),inset 0 1px 0 rgba(255,255,255,.07);
  position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;
  background:linear-gradient(90deg,transparent,rgba(255,255,255,.15),transparent)}
.hero{display:flex;flex-direction:column;align-items:center;padding:36px 16px 28px}
.circle{width:220px;height:220px;border-radius:50%;
  background:radial-gradient(circle at 32% 30%,rgba(234,221,255,.45) 0%,transparent 50%),
    radial-gradient(circle at 70% 75%,rgba(79,55,139,.6) 0%,transparent 60%),
    radial-gradient(circle at 50% 50%,#3a2960 0%,#1a0f30 80%);
  display:flex;align-items:center;justify-content:center;
  transition:transform .4s cubic-bezier(.3,.7,.4,1),box-shadow .4s;
  box-shadow:inset 0 2px 24px rgba(255,255,255,.08),inset 0 -30px 60px rgba(0,0,0,.4),
    0 8px 32px rgba(124,77,255,.25),0 0 0 1px rgba(255,255,255,.06);
  position:relative}
.circle::before{content:'';position:absolute;inset:8px;border-radius:50%;
  background:linear-gradient(135deg,transparent 40%,rgba(255,255,255,.06) 50%,transparent 60%);
  pointer-events:none}
.circle.active{box-shadow:inset 0 2px 24px rgba(255,255,255,.12),inset 0 -30px 60px rgba(0,0,0,.4),
    0 0 80px 8px rgba(208,188,255,.35),0 0 0 1px rgba(208,188,255,.2)}
.bpm{font-size:3.8rem;font-weight:200;line-height:1;text-align:center;
  background:linear-gradient(180deg,#fff 0%,#d0bcff 100%);-webkit-background-clip:text;background-clip:text;color:transparent;
  text-shadow:0 2px 20px rgba(208,188,255,.3)}
.bpm-u{font-size:.72rem;color:var(--on-variant);margin-top:8px;letter-spacing:1px;text-align:center;text-transform:uppercase}
.phase-lbl{margin-top:18px;min-height:22px;font-size:.82rem;color:var(--on-primary-c);
  letter-spacing:1.5px;font-weight:600;text-transform:uppercase;opacity:0;transition:opacity .3s;
  text-align:center;max-width:280px;line-height:1.4;padding:0 12px}
.phase-lbl.on{opacity:1}
.posture{margin-top:14px;padding:9px 20px;border-radius:14px;
  background:rgba(74,68,88,.5);color:var(--on-secondary-c);
  font-size:.85rem;font-weight:500;user-select:none;
  border:1px solid var(--glass-hi);
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.stat{background:var(--glass);backdrop-filter:blur(20px) saturate(160%);-webkit-backdrop-filter:blur(20px) saturate(160%);
  border-radius:var(--r-m);padding:18px 12px;text-align:center;border:1px solid var(--glass-hi)}
.stat-v{font-size:1.6rem;font-weight:400;line-height:1;color:#fff}
.stat-l{font-size:.66rem;color:var(--on-variant);margin-top:8px;letter-spacing:.8px;text-transform:uppercase;font-weight:600}
.row{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:14px}
.btn{display:flex;align-items:center;justify-content:center;gap:8px;padding:15px 18px;border-radius:999px;border:none;
  font-family:inherit;font-size:.88rem;font-weight:500;cursor:pointer;transition:all .2s;letter-spacing:.3px;
  position:relative;overflow:hidden}
.btn::after{content:'';position:absolute;inset:0;border-radius:inherit;
  background:linear-gradient(180deg,rgba(255,255,255,.12),transparent 50%);pointer-events:none}
.btn:active{transform:scale(.97)}
.btn-f{background:var(--primary);color:var(--on-primary);box-shadow:0 4px 16px rgba(208,188,255,.3)}
.btn-t{background:rgba(74,68,88,.7);color:var(--on-secondary-c);
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border:1px solid var(--glass-hi)}
.btn-o{background:transparent;color:var(--primary);border:1px solid rgba(208,188,255,.3);
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}
.btn-e{background:rgba(242,184,181,.85);color:var(--on-error)}
canvas{width:100%;height:140px;background:rgba(14,12,18,.5);
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
  border-radius:var(--r-s);display:block;margin-top:14px;border:1px solid var(--glass-hi)}
.pats{display:flex;gap:10px;margin-top:14px}
.pat{flex:1;padding:14px 4px;border-radius:14px;border:1px solid var(--glass-hi);
  background:rgba(255,255,255,.03);color:var(--on-surface);font-family:inherit;font-size:.88rem;cursor:pointer;transition:all .2s;
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}
.pat.on{background:linear-gradient(135deg,rgba(208,188,255,.25),rgba(124,77,255,.15));
  color:#fff;border-color:rgba(208,188,255,.4);font-weight:500;
  box-shadow:0 4px 12px rgba(124,77,255,.2)}
.label{font-size:.7rem;color:var(--on-variant);text-transform:uppercase;letter-spacing:1.4px;font-weight:600}
details summary{list-style:none;cursor:pointer;padding:6px 0;font-size:.9rem;color:var(--on-variant);display:flex;align-items:center;font-weight:500}
details summary::after{content:'>';margin-left:auto;transition:transform .25s;color:var(--primary);transform:rotate(90deg)}
details[open] summary::after{transform:rotate(270deg)}
.kv{display:flex;justify-content:space-between;padding:11px 0;font-size:.88rem;border-bottom:1px solid rgba(255,255,255,.05)}
.kv:last-child{border-bottom:none}
.kv .k{color:var(--on-variant)}
.kv .v{font-family:'SF Mono','Menlo',monospace;color:#fff}
.prof{padding:16px;border-radius:var(--r-s);background:rgba(255,255,255,.04);margin-top:10px;border:1px solid var(--glass-hi)}
.prof .n{font-weight:500;display:flex;align-items:center;gap:10px}
.dot{width:9px;height:9px;border-radius:50%;background:rgba(255,255,255,.2);box-shadow:0 0 8px rgba(255,255,255,.1)}
.dot.ok{background:#7ed957;box-shadow:0 0 12px rgba(126,217,87,.6)}
.prof .d{margin-top:8px;font-size:.78rem;color:var(--on-variant);font-family:monospace}
.section-l{margin:22px 0 8px;color:var(--on-variant);font-size:.74rem;letter-spacing:1.2px;font-weight:600;text-transform:uppercase}
.phase-bar{margin-top:8px;height:4px;width:160px;border-radius:2px;
  background:rgba(255,255,255,.08);overflow:hidden;opacity:0;transition:opacity .3s}
.phase-bar.on{opacity:1}
.phase-fill{height:100%;background:linear-gradient(90deg,#d0bcff,#ff6b9d);
  border-radius:2px;transition:width .15s linear;box-shadow:0 0 8px rgba(208,188,255,.5)}
.toast{position:fixed;bottom:28px;left:50%;transform:translateX(-50%) translateY(140px);
  background:var(--glass);backdrop-filter:blur(20px) saturate(180%);-webkit-backdrop-filter:blur(20px) saturate(180%);
  color:var(--on-surface);padding:14px 26px;border-radius:var(--r-m);
  transition:transform .35s cubic-bezier(.3,.7,.4,1);
  z-index:100;border:1px solid var(--glass-hi);
  box-shadow:0 12px 40px var(--glass-lo);font-size:.88rem;letter-spacing:.3px}
.toast.show{transform:translateX(-50%) translateY(0)}
/* ===== Calibration card (Material 3 step-by-step guidance) ===== */
.cal-card{
  background:linear-gradient(140deg,rgba(124,77,255,.22),rgba(208,188,255,.06) 50%,rgba(255,107,157,.14));
  border:1px solid rgba(208,188,255,.32);padding:26px 22px 22px;text-align:center;
}
.cal-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:20px;gap:12px}
.cal-type{font-size:.66rem;font-weight:700;letter-spacing:1.6px;color:var(--on-primary);
  background:var(--primary);padding:6px 12px;border-radius:999px;
  box-shadow:0 2px 10px rgba(208,188,255,.3)}
.cal-time{font-size:.78rem;color:var(--on-variant);font-family:'SF Mono','Menlo',monospace;
  letter-spacing:.5px}
.cal-stepper{display:flex;align-items:center;justify-content:center;margin-bottom:24px;padding:0 4px}
.cal-step{
  width:32px;height:32px;border-radius:50%;flex-shrink:0;
  display:flex;align-items:center;justify-content:center;
  background:rgba(255,255,255,.04);border:2px solid rgba(255,255,255,.12);
  color:var(--on-variant);font-size:.82rem;font-weight:600;
  transition:all .4s cubic-bezier(.3,.7,.4,1);
}
.cal-step.done{background:rgba(126,217,87,.18);border-color:#7ed957;color:#c4f59a}
.cal-step.current{
  background:linear-gradient(135deg,#d0bcff,#7c4dff);
  border-color:#d0bcff;color:#1a0f30;transform:scale(1.18);
  box-shadow:0 0 0 4px rgba(208,188,255,.18),0 4px 18px rgba(124,77,255,.45);
  animation:calStepPulse 1.8s ease-in-out infinite;
}
@keyframes calStepPulse{
  0%,100%{box-shadow:0 0 0 4px rgba(208,188,255,.18),0 4px 18px rgba(124,77,255,.45)}
  50%{box-shadow:0 0 0 9px rgba(208,188,255,.05),0 4px 28px rgba(124,77,255,.65)}
}
.cal-line{
  flex:1;height:2px;max-width:48px;min-width:14px;margin:0 6px;
  background:rgba(255,255,255,.12);transition:background .4s;
}
.cal-line.done{background:#7ed957}
.cal-phase{
  font-size:1.7rem;font-weight:300;letter-spacing:4px;line-height:1;
  background:linear-gradient(135deg,#fff,#d0bcff);
  -webkit-background-clip:text;background-clip:text;color:transparent;
  margin-bottom:14px;
}
.cal-instr{
  font-size:.95rem;line-height:1.5;color:var(--on-surface);
  margin:0 auto 22px;max-width:300px;min-height:42px;padding:0 8px;
  display:flex;align-items:center;justify-content:center;
}
.cal-progress{
  height:5px;border-radius:3px;background:rgba(255,255,255,.07);
  margin:0 0 12px;overflow:hidden;
}
.cal-progress-fill{
  height:100%;width:0;border-radius:3px;
  background:linear-gradient(90deg,#d0bcff,#ff6b9d);
  box-shadow:0 0 10px rgba(208,188,255,.5);
  transition:width .25s cubic-bezier(.3,.7,.4,1);
}
.cal-meta{
  display:flex;justify-content:space-between;
  font-size:.74rem;color:var(--on-variant);letter-spacing:.4px;padding:2px 4px 0;
}
.cal-posture{margin-top:18px;display:flex;justify-content:center}
/* ===== Recalibration countdown card (5s prep before cal starts) ===== */
.cdn-card{
  background:linear-gradient(135deg,rgba(124,77,255,.24),rgba(208,188,255,.06) 60%,rgba(255,107,157,.16));
  border:1px solid rgba(208,188,255,.34);padding:34px 22px 26px;text-align:center;
}
.cdn-title{font-size:.7rem;letter-spacing:1.8px;font-weight:700;
  color:var(--primary);text-transform:uppercase;margin-bottom:8px}
.cdn-num{font-size:5.5rem;font-weight:200;line-height:1;
  background:linear-gradient(180deg,#fff,#d0bcff);
  -webkit-background-clip:text;background-clip:text;color:transparent;
  margin:14px 0 18px;animation:cdnPulse 1s ease-in-out infinite}
@keyframes cdnPulse{
  0%,100%{transform:scale(1)}
  50%{transform:scale(1.10)}
}
.cdn-instr{font-size:.95rem;line-height:1.5;color:var(--on-surface);
  max-width:300px;margin:0 auto 22px;padding:0 8px}
.cdn-cancel{max-width:160px;margin:0 auto;display:block}
</style></head>
<body>
<div class="bg"></div>
<div class="bar"><h1>Aria</h1><span id="st" class="pill">—</span></div>
<div class="wrap">
  <div id="countdownCard" class="card cdn-card" style="display:none">
    <div class="cdn-title">Preparing calibration</div>
    <div id="cdnNum" class="cdn-num">5</div>
    <div class="cdn-instr">Get into your current posture and stay still. Calibration will begin in a few seconds.</div>
    <button class="btn btn-t cdn-cancel" onclick="cancelCountdown()">Cancel</button>
  </div>
  <div id="calCard" class="card cal-card" style="display:none">
    <div class="cal-head">
      <span id="calType" class="cal-type">CALIBRATION</span>
      <span id="calTime" class="cal-time">—</span>
    </div>
    <div id="calStepper" class="cal-stepper"></div>
    <div id="calPhase" class="cal-phase">—</div>
    <div id="calInstr" class="cal-instr">—</div>
    <div class="cal-progress"><div id="calProgFill" class="cal-progress-fill"></div></div>
    <div class="cal-meta">
      <span id="calStepLabel">Step — of —</span>
      <span id="calRemaining">— left</span>
    </div>
    <div class="cal-posture"><div id="poCal" class="posture">—</div></div>
  </div>
  <div id="heroCard" class="card hero">
    <div id="circle" class="circle"><div>
      <div id="bpm" class="bpm">—</div>
      <div id="bpmU" class="bpm-u">breaths per minute</div>
    </div></div>
    <div id="phaseLbl" class="phase-lbl">—</div>
    <div id="phaseBar" class="phase-bar"><div id="phaseFill" class="phase-fill" style="width:0"></div></div>
    <div id="po" class="posture">—</div>
  </div>
  <div id="statsCard" class="stats">
    <div class="stat"><div id="rg" class="stat-v">—</div><div class="stat-l">Regularity</div></div>
    <div class="stat"><div id="br" class="stat-v">0</div><div class="stat-l">Breaths</div></div>
    <div class="stat"><div id="du" class="stat-v">0:00</div><div class="stat-l">Session</div></div>
  </div>
  <div id="controlsCard" class="card">
    <div class="label">Controls</div>
    <div class="row">
      <button id="bp" class="btn btn-f" onclick="cmd('start')">Start session</button>
      <button class="btn btn-t" onclick="cmd('relax')">Toggle Relax</button>
    </div>
  </div>
  <div class="card">
    <div class="label">Live pressure</div>
    <canvas id="ch"></canvas>
  </div>
  <div class="card">
    <div class="label">Relax pattern</div>
    <div class="pats">
      <button class="pat" onclick="setp(0)">3-3</button>
      <button class="pat" onclick="setp(1)">4-7-8</button>
      <button class="pat" onclick="setp(2)">Box</button>
    </div>
  </div>
  <div class="card"><details>
    <summary>Advanced</summary>
    <div class="section-l">Active thresholds</div>
    <div class="kv"><span class="k">Baseline (kPa)</span><span id="bl" class="v">—</span></div>
    <div class="kv"><span class="k">Breath threshold</span><span id="bt" class="v">—</span></div>
    <div class="kv"><span class="k">Pinch threshold</span><span id="pt" class="v">—</span></div>
    <div class="kv"><span class="k">Live dp</span><span id="dp" class="v">—</span></div>
    <div class="section-l">Profiles</div>
    <div id="prof"></div>
    <div class="section-l">Force events</div>
    <div class="row">
      <button class="btn btn-o" onclick="cmd('hyper')">Hyperventilation</button>
      <button class="btn btn-o" onclick="cmd('apnoea')">Apnoea</button>
    </div>
    <div class="section-l">Calibration</div>
    <div class="row">
      <button class="btn btn-t" onclick="showRecalCountdown()">Recalibrate</button>
      <button class="btn btn-e" onclick="wipe()">Wipe profiles</button>
    </div>
    <div class="section-l">Session counters</div>
    <div class="kv"><span class="k">Apnoea events</span><span id="ap" class="v">0</span></div>
    <div class="kv"><span class="k">Hyperventilation events</span><span id="hy" class="v">0</span></div>
  </details></div>
</div>
<div id="ts" class="toast"></div>
<script>
const H=Array(120).fill(0);
let busy=false;
async function fetchData(){
  if(busy)return;busy=true;
  try{const r=await fetch('/data',{cache:'no-store'});const d=await r.json();render(d);}
  catch(e){}finally{busy=false;}
}
async function cmd(action,params){
  let url='/cmd?a='+action;
  if(params&&params.p!==undefined)url+='&p='+params.p;
  toast(action.toUpperCase());
  try{
    const r=await fetch(url,{cache:'no-store'});
    // /cmd now returns the full state — render immediately, no follow-up poll
    const d=await r.json();
    if(d&&d.state)render(d);
  }catch(e){}
}
// Self-rescheduling loop: never queues overlapping requests, adapts to Arduino's pace
(async function pollLoop(){
  while(true){
    await fetchData();
    await new Promise(r=>setTimeout(r,150));
  }
})();
function setp(i){cmd('pat',{p:i});}
function wipe(){if(confirm('Wipe all calibration profiles?'))cmd('wipe');}
// ----- Recalibration countdown: scroll to top, show 5s countdown, then trigger /cmd?a=recal
let countdownTimer=null;
function showRecalCountdown(){
  window.scrollTo({top:0,behavior:'smooth'});
  if(countdownTimer)clearInterval(countdownTimer);
  let left=5;
  const numEl=document.getElementById('cdnNum');
  const card=document.getElementById('countdownCard');
  numEl.textContent=left;
  card.style.display='block';
  countdownTimer=setInterval(()=>{
    left--;
    if(left>0){numEl.textContent=left;}
    else{
      clearInterval(countdownTimer);countdownTimer=null;
      card.style.display='none';
      cmd('recal');
    }
  },1000);
}
function cancelCountdown(){
  if(countdownTimer){clearInterval(countdownTimer);countdownTimer=null;}
  document.getElementById('countdownCard').style.display='none';
  toast('Cancelled');
}
function fmt(s){if(!s)return'0:00';const m=Math.floor(s/60);const sec=s%60;return m+':'+sec.toString().padStart(2,'0');}
function draw(){
  const c=document.getElementById('ch');const x=c.getContext('2d');
  c.width=c.clientWidth;c.height=c.clientHeight;x.clearRect(0,0,c.width,c.height);
  x.strokeStyle='rgba(208,188,255,.12)';x.lineWidth=1;x.beginPath();x.moveTo(0,c.height/2);x.lineTo(c.width,c.height/2);x.stroke();
  const m=Math.max(...H.map(Math.abs),.05);
  const grad=x.createLinearGradient(0,0,0,c.height);
  grad.addColorStop(0,'rgba(208,188,255,.25)');grad.addColorStop(1,'rgba(208,188,255,0)');
  x.fillStyle=grad;x.beginPath();x.moveTo(0,c.height/2);
  for(let i=0;i<H.length;i++){const px=(i/(H.length-1))*c.width;const py=c.height/2-(H[i]/m)*(c.height/2-6);x.lineTo(px,py);}
  x.lineTo(c.width,c.height/2);x.fill();
  x.strokeStyle='#d0bcff';x.lineWidth=2.2;x.shadowColor='rgba(208,188,255,.5)';x.shadowBlur=8;x.beginPath();
  for(let i=0;i<H.length;i++){const px=(i/(H.length-1))*c.width;const py=c.height/2-(H[i]/m)*(c.height/2-6);if(i===0)x.moveTo(px,py);else x.lineTo(px,py);}
  x.stroke();x.shadowBlur=0;
}
function render(d){
  const sp=document.getElementById('st');sp.textContent=d.state;sp.className='pill s-'+d.state;
  document.getElementById('bpm').textContent=d.bpm?d.bpm.toFixed(1):'—';
  document.getElementById('po').textContent=d.posture||'—';
  document.getElementById('rg').textContent=d.reg+'%';
  document.getElementById('br').textContent=d.breaths;
  document.getElementById('du').textContent=fmt(d.duration);
  document.getElementById('ap').textContent=d.apnoeas;
  document.getElementById('hy').textContent=d.hypers;
  document.getElementById('bl').textContent=(d.baseline||0).toFixed(3);
  document.getElementById('bt').textContent=(d.bThr||0).toFixed(3);
  document.getElementById('pt').textContent=(d.pThr||0).toFixed(3);
  document.getElementById('dp').textContent=(d.dp||0).toFixed(3);
  const bp=document.getElementById('bp');
  if(d.state==='OFF'){bp.textContent='Start session';bp.className='btn btn-f';}
  else if(d.state==='MONITORING'){bp.textContent='Stop session';bp.className='btn btn-t';}
  else if(d.state==='RELAX'){bp.textContent='Exit relax';bp.className='btn btn-t';}
  else if(d.state==='HYPERVENT_RESCUE'){bp.textContent='Dismiss rescue';bp.className='btn btn-t';}
  else if(d.state==='APNOEA_ALERT'){bp.textContent='Dismiss alert';bp.className='btn btn-t';}
  else{bp.textContent='Stop ('+d.state+')';bp.className='btn btn-t';}
  const c=document.getElementById('circle');
  const sc=1+Math.max(-1,Math.min(1,(d.dp||0)/.08))*.13;
  c.style.transform='scale('+sc+')';
  if(d.breathing)c.classList.add('active');else c.classList.remove('active');
  // ----- Calibration card vs. regular hero -----
  const RECAL_INFO={
    // Silent recal (3 phases, 20s total)
    'WAIT':            {num:1,total:3,name:'SETTLING', instr:'Hold still — sensor is stabilizing',                offset:0},
    'BASELINE':        {num:2,total:3,name:'BASELINE', instr:'Hold still — measuring your resting pressure',      offset:3000},
    'BREATH_AMP':      {num:3,total:3,name:'BREATHING',instr:'Breathe normally for the rest of the time',         offset:8000},
    // Initial guided cal (4 phases, 38s total)
    'INITCAL_SETTLE':  {num:1,total:4,name:'SETTLING', instr:'Get comfortable in your current posture and hold still', offset:0},
    'INITCAL_BASELINE':{num:2,total:4,name:'BASELINE', instr:'Stay still — measuring your resting chest pressure', offset:3000},
    'INITCAL_BREATH':  {num:3,total:4,name:'BREATHING',instr:'Breathe normally and slightly deeper than usual',   offset:8000},
    'INITCAL_PINCH':   {num:4,total:4,name:'PINCH',    instr:'Pinch the silicone tube firmly, 3 to 5 times',      offset:23000}
  };
  const RECAL_TOTALS={SILENT:20000,INITIAL:38000};
  const calCard=document.getElementById('calCard');
  const heroCard=document.getElementById('heroCard');
  const statsCard=document.getElementById('statsCard');
  const controlsCard=document.getElementById('controlsCard');
  const isCal=d.state==='RECALIBRATING'&&d.recalPhase&&d.recalPhase!=='NONE'&&RECAL_INFO[d.recalPhase];
  if(isCal){
    const info=RECAL_INFO[d.recalPhase];
    const isInitial=d.recalType==='INITIAL';
    const total=isInitial?RECAL_TOTALS.INITIAL:RECAL_TOTALS.SILENT;
    const elapsed=d.recalElapsed||0;
    const phaseTotal=d.recalPhaseTotal||1;
    const phaseLeft=Math.max(0,Math.ceil((phaseTotal-elapsed)/1000));
    const overallLeft=Math.max(0,Math.ceil((total-info.offset-elapsed)/1000));
    document.getElementById('calType').textContent=isInitial?'INITIAL CALIBRATION':'RECALIBRATION';
    document.getElementById('calTime').textContent='~'+overallLeft+'s total';
    document.getElementById('calPhase').textContent=info.name;
    document.getElementById('calInstr').textContent=info.instr;
    // Build stepper
    let html='';
    for(let i=1;i<=info.total;i++){
      const cls=i<info.num?'done':(i===info.num?'current':'');
      html+='<div class="cal-step '+cls+'">'+(i<info.num?'✓':i)+'</div>';
      if(i<info.total) html+='<div class="cal-line '+(i<info.num?'done':'')+'"></div>';
    }
    document.getElementById('calStepper').innerHTML=html;
    // Progress bar tracks overall calibration completion
    const overall=Math.min(1,(info.offset+elapsed)/total);
    document.getElementById('calProgFill').style.width=(overall*100)+'%';
    document.getElementById('calStepLabel').textContent='Step '+info.num+' of '+info.total;
    document.getElementById('calRemaining').textContent=phaseLeft+'s left in this step';
    document.getElementById('poCal').textContent=d.posture||'—';
    calCard.style.display='block';
    heroCard.style.display='none';
    statsCard.style.display='none';
    controlsCard.style.display='none';
  } else {
    calCard.style.display='none';
    heroCard.style.display='flex';
    statsCard.style.display='grid';
    controlsCard.style.display='block';
  }
  // ----- Hero phase indicator (RELAX only — recal handled by calCard) -----
  const bpmEl=document.getElementById('bpm');
  const bpmUEl=document.getElementById('bpmU');
  const pl=document.getElementById('phaseLbl');
  const pb=document.getElementById('phaseBar');
  const pf=document.getElementById('phaseFill');
  bpmEl.classList.remove('recal');bpmUEl.classList.remove('recal');
  bpmEl.textContent=d.bpm?d.bpm.toFixed(1):'—';
  bpmUEl.textContent='breaths per minute';
  if(d.state==='RELAX'&&d.phaseLabel){
    pl.textContent=d.phaseLabel;pl.classList.add('on');
    pb.classList.add('on');pf.style.width=((d.phaseProg||0)*100)+'%';
  } else {
    pl.classList.remove('on');pb.classList.remove('on');
  }
  document.querySelectorAll('.pat').forEach((p,i)=>p.classList.toggle('on',i===d.patternIdx));
  document.getElementById('prof').innerHTML=(d.profiles||[]).map(p=>
    '<div class="prof"><div class="n"><span class="dot '+(p.cal?'ok':'')+'"></span>'+p.name+'</div>'+
    '<div class="d">'+(p.cal?'bThr='+p.bThr.toFixed(3)+' pThr='+p.pThr.toFixed(3):'not calibrated')+'</div></div>'
  ).join('');
  H.shift();H.push(d.dp||0);draw();
}
function toast(m){const t=document.getElementById('ts');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),1800);}
addEventListener('resize',draw);
</script></body></html>)PAGE";

void sendHtmlPage(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  const char* p = HTML_PAGE;
  char buf[129];
  while (*p) {
    int i = 0;
    while (i < 128 && *p) buf[i++] = *p++;
    buf[i] = 0;
    client.print(buf);
  }
}

void writeStateJson(WiFiClient& client) {
  client.print(F("{\"state\":\""));     client.print(stateToString(currentState));
  client.print(F("\",\"posture\":\"")); client.print(postureToString(currentPosture));
  client.print(F("\",\"bpm\":"));       client.print(currentBpm, 1);
  client.print(F(",\"reg\":"));         client.print((int)computeRegularity());
  client.print(F(",\"dp\":"));          client.print(dp_kpa - dp_baseline, 4);
  client.print(F(",\"baseline\":"));    client.print(dp_baseline, 3);
  client.print(F(",\"bThr\":"));        client.print(breathThreshold, 4);
  client.print(F(",\"pThr\":"));        client.print(pinchThreshold, 4);
  client.print(F(",\"breaths\":"));     client.print(totalBreaths);
  unsigned long dur = (currentState == STATE_OFF) ? 0 : (millis() - sessionStartTime) / 1000;
  client.print(F(",\"duration\":"));    client.print(dur);
  client.print(F(",\"apnoeas\":"));     client.print(apnoeaEvents);
  client.print(F(",\"hypers\":"));      client.print(hyperventEvents);
  client.print(F(",\"breathing\":"));   client.print(inBreath ? "true" : "false");
  client.print(F(",\"patternIdx\":"));  client.print(currentRelaxPattern);
  client.print(F(",\"phaseIdx\":"));    client.print(currentRelaxPhase);
  client.print(F(",\"phaseLabel\":\""));
  if (currentState == STATE_RELAX) client.print(PATTERNS[currentRelaxPattern].phaseLabels[currentRelaxPhase]);
  client.print(F("\",\"phaseProg\":")); client.print(currentRelaxProg, 3);
  // ----- Recalibration progress (for guided UI in dashboard) -----
  client.print(F(",\"recalPhase\":\""));
  switch (recalPhase) {
    case RECAL_WAIT:       client.print(F("WAIT"));            break;
    case RECAL_BASELINE:   client.print(F("BASELINE"));        break;
    case RECAL_BREATH_AMP: client.print(F("BREATH_AMP"));      break;
    case INITCAL_SETTLE:   client.print(F("INITCAL_SETTLE"));   break;
    case INITCAL_BASELINE: client.print(F("INITCAL_BASELINE")); break;
    case INITCAL_BREATH:   client.print(F("INITCAL_BREATH"));   break;
    case INITCAL_PINCH:    client.print(F("INITCAL_PINCH"));    break;
    default:               client.print(F("NONE"));
  }
  client.print(F("\",\"recalType\":\""));
  if (recalPhase >= INITCAL_SETTLE)  client.print(F("INITIAL"));
  else if (recalPhase != RECAL_NONE) client.print(F("SILENT"));
  else                               client.print(F("NONE"));
  client.print(F("\",\"recalElapsed\":"));
  client.print((currentState == STATE_RECALIBRATING && recalPhase != RECAL_NONE) ? (millis() - recalPhaseStart) : 0UL);
  client.print(F(",\"recalPhaseTotal\":"));
  switch (recalPhase) {
    case RECAL_WAIT:       client.print(RECAL_WAIT_MS);       break;
    case RECAL_BASELINE:   client.print(RECAL_BASELINE_MS);   break;
    case RECAL_BREATH_AMP: client.print(RECAL_BREATH_AMP_MS); break;
    case INITCAL_SETTLE:   client.print(INITCAL_SETTLE_MS);   break;
    case INITCAL_BASELINE: client.print(INITCAL_BASELINE_MS); break;
    case INITCAL_BREATH:   client.print(INITCAL_BREATH_MS);   break;
    case INITCAL_PINCH:    client.print(INITCAL_PINCH_MS);    break;
    default:               client.print(0);
  }
  client.print(F(",\"profiles\":[{\"name\":\"ERETTO\",\"cal\":"));
  client.print(profiles[POSTURE_UPRIGHT].calibrated ? "true" : "false");
  client.print(F(",\"bThr\":")); client.print(profiles[POSTURE_UPRIGHT].breathThr, 3);
  client.print(F(",\"pThr\":")); client.print(profiles[POSTURE_UPRIGHT].pinchThr, 3);
  client.print(F("},{\"name\":\"SDRAIATO\",\"cal\":"));
  client.print(profiles[POSTURE_LYING].calibrated ? "true" : "false");
  client.print(F(",\"bThr\":")); client.print(profiles[POSTURE_LYING].breathThr, 3);
  client.print(F(",\"pThr\":")); client.print(profiles[POSTURE_LYING].pinchThr, 3);
  client.println(F("}]}"));
}

void sendJson(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Cache-Control: no-cache"));
  client.println(F("Connection: close"));
  client.println();
  writeStateJson(client);
}

void handleCmd(WiFiClient& client, const char* line) {
  char action[16] = {0};
  const char* a = strstr(line, "a=");
  if (a) {
    a += 2;
    int i = 0;
    while (*a && *a != '&' && *a != ' ' && i < 15) action[i++] = *a++;
    action[i] = 0;
  }
  if (!strcmp(action, "start") || !strcmp(action, "stop")) {
    handleDoublePinch();
  } else if (!strcmp(action, "relax")) {
    if (currentState == STATE_MONITORING)     changeState(STATE_RELAX);
    else if (currentState == STATE_RELAX)     changeState(STATE_MONITORING);
  } else if (!strcmp(action, "hyper")) {
    if (currentState == STATE_MONITORING) changeState(STATE_HYPERVENT_RESCUE);
  } else if (!strcmp(action, "apnoea")) {
    if (currentState == STATE_MONITORING) changeState(STATE_APNOEA_ALERT);
  } else if (!strcmp(action, "recal")) {
    if (currentState == STATE_MONITORING) startSilentRecal();
    else if (currentState == STATE_OFF)   startInitialCalibration();
  } else if (!strcmp(action, "wipe")) {
    if (currentState == STATE_RECALIBRATING) {
      Serial.println(F("[WIPE] Ignored — calibration in progress"));
    } else {
      EEPROM.put(0, (uint32_t)0xDEADBEEF);
      for (int i = 0; i < 4; i++) profiles[i].calibrated = false;
      Serial.println(F("[FLASH] Profiles wiped via web (RAM + Flash)"));
    }
  } else if (!strcmp(action, "pat")) {
    const char* p = strstr(line, "p=");
    if (p) {
      int idx = atoi(p + 2);
      if (idx >= 0 && idx < 3) {
        currentRelaxPattern = idx;
        if (currentState == STATE_RELAX) startRelaxCycle();
        Serial.print(F("[RELAX] (web) -> "));
        Serial.println(PATTERNS[idx].name);
      }
    }
  }
  // Return the FULL state — so the dashboard renders the result immediately
  // without needing a follow-up /data request.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Cache-Control: no-cache"));
  client.println(F("Connection: close"));
  client.println();
  writeStateJson(client);
}

void handleWiFi() {
  WiFiClient client = wifiServer.available();
  if (!client) return;
  unsigned long timeout = millis() + 300;
  char line[128]; int idx = 0; bool eol = false;
  while (client.connected() && millis() < timeout && !eol) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') eol = true;
      else if (c != '\r' && idx < 127) line[idx++] = c;
    }
  }
  line[idx] = 0;
  while (client.connected() && client.available() && millis() < timeout) client.read();
  if      (strstr(line, "GET /data") != NULL) sendJson(client);
  else if (strstr(line, "GET /cmd")  != NULL) handleCmd(client, line);
  else                                        sendHtmlPage(client);
  delay(5);
  client.stop();
}

void setupWiFi() {
  Serial.print(F("[WIFI] AP "));
  Serial.println(AP_SSID);
  if (WiFi.beginAP(AP_SSID, AP_PASS) != WL_AP_LISTENING) {
    Serial.println(F("[WIFI] AP failed"));
    return;
  }
  wifiServer.begin();
  delay(500);
  Serial.print(F("[WIFI] IP: "));
  Serial.println(WiFi.localIP());
}

// ============== BOOT POSTURE DETECTION ==============
void detectBootPosture() {
  for (int i = 0; i < 25; i++) {
    mpuRead();
    updateActivityBuffer();
    delay(20);
  }
  float absX = fabs(ax_g), absY = fabs(ay_g), absZ = fabs(az_g);
  if (absZ >= absX && absZ >= absY) currentPosture = POSTURE_UPRIGHT;
  else                              currentPosture = POSTURE_LYING;
  rawPosture    = currentPosture;
  postureStable = POSTURE_HYSTERESIS;
  Serial.print(F("[BOOT] Detected posture: "));
  Serial.println(postureToString(currentPosture));
}

// ============== SETUP & LOOP ==============
void setup() {
  pinMode(PIN_MOTOR, OUTPUT); analogWrite(PIN_MOTOR, 0);
  pinMode(LED_PIN, OUTPUT);   digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("\n=== ARIA v7 ==="));
  Serial.println(F("Tacconi & Di Staso"));
  Serial.println(F("Type ? for serial commands"));

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  if (Wire.endTransmission() != 0) Serial.println(F("[MPU] NOT RESPONDING"));
  else {
    Serial.println(F("[MPU] OK"));
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission();
  }
  analogReadResolution(10);
  setupWiFi();
  loadProfilesFromFlash();
  detectBootPosture();

  if (profiles[currentPosture].calibrated) {
    dp_baseline     = profiles[currentPosture].baseline;
    breathThreshold = profiles[currentPosture].breathThr;
    pinchThreshold  = profiles[currentPosture].pinchThr;
    Serial.print(F("[BOOT] Profile restored for "));
    Serial.println(postureToString(currentPosture));
  } else {
    Serial.println(F("[BOOT] No profile for this posture, starting guided calibration"));
    Serial.println(F("[BOOT] Connect to the WiFi AP to follow the steps in the dashboard"));
    startInitialCalibration();
  }

  Serial.println(F("\nDouble-pinch (or send 'o') to start a session."));
  lastStateChange   = millis();
  lastPostureChange = millis();
  sessionStartTime  = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    mpuRead();
    updateActivityBuffer();
    updatePosture();
    readPressure();
    dp_baseline = dp_baseline * (1.0f - BASELINE_ALPHA) + dp_kpa * BASELINE_ALPHA;
    detectPinch();
    if (currentState == STATE_MONITORING) {
      if (currentPosture != POSTURE_MOVING) detectBreath();
      checkApnoea();
      checkHyperventilation();
    } else if (currentState == STATE_RELAX) {
      relaxTick();
    } else if (currentState == STATE_HYPERVENT_RESCUE) {
      runHyperventRescue();
    } else if (currentState == STATE_APNOEA_ALERT) {
      detectBreath();
      checkApnoeaResume();
    } else if (currentState == STATE_RECALIBRATING) {
      runRecalStep();
    }
  }
  updateLed();
  handleSerial();
  handleWiFi();
  if (now - lastStatusPrint >= STATUS_PRINT_MS && currentState != STATE_OFF) {
    lastStatusPrint = now;
    printStatus();
  }
}
