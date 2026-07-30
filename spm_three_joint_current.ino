/*
  SPM Three-Joint Simultaneous Current Control
  Teensy 4.1 + 3 VESCs + 3 Lamprey2 PWM Encoders

  All three joints run their position PID simultaneously.
  Output is setCurrent() — smooth, compliant, no RPM fighting.
  Encoder error is always computed in raw encoder coordinates using shortest-path wrapping.

  This is the direct stepping stone to IK:
    Python IK math → $S / MOVEALL joint angles → this firmware holds position

  Physical mapping (confirmed):
    J1 / M1 / Serial1 → E3 / pin 4
    J2 / M2 / Serial2 → E2 / pin 3
    J3 / M3 / Serial7 → E1 / pin 2

  Serial commands (115200 baud):
    Telemetry prints automatically at TELEM_HZ and appears in the Python terminal.
    ENC_RAW reports physical encoders E1/E2/E3. ENC_JOINT reports J1/J2/J3 order.

    STOP                              release all current immediately
    ENABLE                            enable all joints
    DISABLE                           disable all joints
    MOVEALL <j1_deg> <j2_deg> <j3_deg>   set all three joint targets
    MOVE <j> <deg>                    set single joint target (1-3)
    CONFIG <kp> <min_a> <max_a> <deadband>
    HOLD                              hold current measured joint position
    GOHOME                            command all joints to fixed encoder home angles
    HB                                heartbeat (resets watchdog timeout)

  Also accepts the live-simulation packet sent by spm_angle_comm.py:
    $S,<j1_deg>,<j2_deg>,<j3_deg>,<enable>*<xor>

  Left-click in the Python sim sends the displayed IK joint angles as an
  S packet. This firmware treats those angles the same as MOVEALL targets
  and uses encoder feedback plus current PID to move each joint there.

  Tuned defaults (100:1 gearbox, 6A stiction threshold):
    kp       = 0.30 A/deg
    minA     = 7.0 A
    maxA     = 12.0 A
    deadband = 2.0 deg  (hysteresis: enter at 2°, exit at 3.6°)

  Example session:
    CONFIG 0.30 7.0 12.0 2.0
    ENABLE
    MOVEALL 20.0 -15.0 10.0
    GOHOME
    STOP
*/

#include <Arduino.h>
#include <VescUart.h>

// ---------------- Encoder pins ----------------
constexpr uint8_t ENC_PIN[3] = {2, 3, 4};

// Logical joint → raw encoder index
//   J1/M1 → E3 / pin 4 → index 2
//   J2/M2 → E2 / pin 3 → index 1
//   J3/M3 → E1 / pin 2 → index 0
constexpr uint8_t JOINT_TO_ENC[3] = {2, 1, 0};
constexpr int8_t  JOINT_SIGN[3]   = {+1, +1, +1};

// Current command sign for each motor/VESC. Use this if the encoder math is
// correct but a motor pushes away from its target. Keep JOINT_SIGN for the
// simulation-angle-to-encoder-angle convention; flip MOTOR_SIGN for motor
// direction/current polarity.
constexpr int8_t  MOTOR_SIGN[3]   = {+1, +1, +1};
// MOTOR_SIGN is now interpreted in RAW ENCODER coordinates:
//   positive error = target encoder angle is positive/CCW from current by the shortest path.
// Flip MOTOR_SIGN[j] if positive current makes that joint encoder move away from a positive error.

// Fixed absolute encoder home angles, in degrees, indexed by JOINT/MOTOR:
//   JOINT_HOME_ENCODER_DEG[0] = J1 / M1 home reading on its encoder, E3 / pin 4
//   JOINT_HOME_ENCODER_DEG[1] = J2 / M2 home reading on its encoder, E2 / pin 3
//   JOINT_HOME_ENCODER_DEG[2] = J3 / M3 home reading on its encoder, E1 / pin 2
//
// This is intentionally in J1/J2/J3 order, because startup homing commands
// motors in joint order. Do NOT enter these as raw E1/E2/E3 order.
constexpr float JOINT_HOME_ENCODER_DEG[3] = {47.5f, 288.0f, 94.5f};
constexpr bool  ENABLE_AT_STARTUP   = true;

// ---------------- VESC UART ports ----------------
constexpr uint8_t N_JOINTS = 3;
constexpr long VESC_BAUD = 115200;

VescUart VESC[N_JOINTS];

HardwareSerial * const VESC_PORT[N_JOINTS] = {
  &Serial1,
  &Serial2,
  &Serial7
};

// ---------------- Timing ----------------
constexpr uint32_t CONTROL_HZ        = 100;
constexpr uint32_t TELEM_HZ          = 10;  // telemetry print rate to Python terminal
constexpr uint32_t LIVE_TIMEOUT_MS   = 200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
constexpr uint32_t STARTUP_HOME_RUN_MS = 15000;
constexpr uint32_t STARTUP_HOME_SETTLED_MS = 750;

// ---------------- Constants ----------------
constexpr float MY_TWO_PI    = 6.28318530718f;
constexpr float MY_RAD_TO_DEG = 57.2957795131f;
// No software joint-angle limit is applied in this firmware.
// Mechanical/workspace limiting should happen in the Python IK/simulation layer.

// Minimum allowed separation between the three proximal-link planes, in degrees.
// The Python simulation enforces the same rule before sending targets. This
// firmware check rejects serial targets that would place two proximal links
// closer than this in the same azimuthal plane.
constexpr float MIN_LINK_CLEARANCE_DEG = 60.0f;
constexpr float JOINT_ETA_DEG[3] = {0.0f, 120.0f, 240.0f};

// ---------------- Encoder ISR state ----------------
volatile uint32_t rise_us[3]   = {0, 0, 0};
volatile uint32_t high_us[3]   = {0, 0, 0};
volatile uint32_t period_us[3] = {0, 0, 0};
volatile uint32_t last_ms[3]   = {0, 0, 0};

// ---------------- Control state ----------------
bool home_valid = true;
bool enabled    = false;

float home_raw_rad[3]  = {0.0f, 0.0f, 0.0f};
float targetDeg[3]     = {0.0f, 0.0f, 0.0f};
float measuredDeg[3]   = {0.0f, 0.0f, 0.0f};
float cmdCurrentA[3]   = {0.0f, 0.0f, 0.0f};
float errDeg[3]        = {0.0f, 0.0f, 0.0f};
bool  measuredOk[3]    = {false, false, false};
bool  settled[3]       = {false, false, false};

// Tunable parameters — sent via CONFIG command
float kpAmpsPerDeg  = 0.30f;
float minCurrentA   = 7.0f;
float maxCurrentA   = 12.0f;
float deadbandDeg   = 2.0f;

uint32_t lastCommandMs = 0;
bool startupHomeActive = false;
uint32_t startupHomeStartMs = 0;
uint32_t startupHomeSettledSinceMs = 0;
bool directHomeMode = false;

// ---------------- Encoder ISR ----------------
template <uint8_t I>
void encISR() {
  uint32_t now = micros();
  if (digitalReadFast(ENC_PIN[I]) == HIGH) {
    if (rise_us[I] != 0) period_us[I] = now - rise_us[I];
    rise_us[I] = now;
  } else {
    if (rise_us[I] != 0) high_us[I] = now - rise_us[I];
    last_ms[I] = millis();
  }
}

// ---------------- Utility ----------------
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


float smoothstep(float x) {
  x = clampf(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

// ---------------- Encoder reading ----------------
float readRawEncoderDegByEnc(uint8_t enc) {
  noInterrupts();
  uint32_t h  = high_us[enc];
  uint32_t p  = period_us[enc];
  uint32_t lm = last_ms[enc];
  interrupts();

  bool live = (millis() - lm) <= LIVE_TIMEOUT_MS;
  if (!live || p == 0 || h == 0 || h >= p) return NAN;

  float duty = clampf((float)h / (float)p, 0.0f, 1.0f);
  return duty * 360.0f;
}

float normalize360(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

float wrap180(float deg) {
  deg = fmodf(deg + 180.0f, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg - 180.0f;
}

float shortestAngleErrorDeg(float targetDegAbs, float currentDegAbs) {
  // Positive means: move the raw encoder angle in the positive direction by
  // this many degrees. This always chooses the shortest path across 0/360.
  return wrap180(normalize360(targetDegAbs) - normalize360(currentDegAbs));
}
float linkPlaneAzimuthDeg(uint8_t jointIdx, float jointDeg) {
  // Matches the Python simulation's proximal-link plane check:
  // az = pi/2 - eta_i + theta_i. The common +90 degree offset cancels out
  // when comparing gaps, so only -eta_i + theta_i matters here.
  return normalize360(-JOINT_ETA_DEG[jointIdx] + jointDeg);
}

bool targetViolatesLinkClearance(float a, float b, float c) {
  float t[3] = {a, b, c};
  float az[3];
  for (uint8_t j = 0; j < N_JOINTS; j++) az[j] = linkPlaneAzimuthDeg(j, t[j]);

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    for (uint8_t j = i + 1; j < N_JOINTS; j++) {
      float gap = fabsf(wrap180(az[i] - az[j]));
      if (gap < MIN_LINK_CLEARANCE_DEG) return true;
    }
  }
  return false;
}


float homeEncoderDegForJoint(uint8_t jointIdx) {
  return normalize360(JOINT_HOME_ENCODER_DEG[jointIdx]);
}

float jointTargetToRawEncoderDeg(uint8_t jointIdx) {
  return normalize360(homeEncoderDegForJoint(jointIdx) +
                      (float)JOINT_SIGN[jointIdx] * targetDeg[jointIdx]);
}

float readEncoderDeg(uint8_t jointIdx) {
  uint8_t enc = JOINT_TO_ENC[jointIdx];
  float absDeg = readRawEncoderDegByEnc(enc);
  if (isnan(absDeg) || !home_valid) return NAN;

  float relDeg = shortestAngleErrorDeg(homeEncoderDegForJoint(jointIdx), absDeg);
  return -(float)JOINT_SIGN[jointIdx] * relDeg;
}

// ---------------- Fixed home initialization ----------------
void loadFixedHome() {
  for (uint8_t j = 0; j < N_JOINTS; j++) {
    uint8_t enc = JOINT_TO_ENC[j];
    float deg = homeEncoderDegForJoint(j);
    home_raw_rad[enc] = deg / MY_RAD_TO_DEG;
  }

  home_valid = true;
  for (uint8_t j = 0; j < N_JOINTS; j++) {
    targetDeg[j] = 0.0f;
    settled[j] = false;
  }
}

// ---------------- Release all motors ----------------
void releaseAll() {
  for (uint8_t i = 0; i < N_JOINTS; i++) {
    VESC[i].setCurrent(0.0f);
    cmdCurrentA[i] = 0.0f;
  }
}

// ---------------- Current command computation ----------------
float computeCurrentCmd(float err, uint8_t j) {
  float absErr = fabsf(err);

  // Hysteresis deadband
  float enterBand = deadbandDeg;
  float exitBand  = deadbandDeg * 1.8f;

  if (settled[j]) {
    if (absErr > exitBand) settled[j] = false;
    else return 0.0f;
  } else {
    if (absErr <= enterBand) {
      settled[j] = true;
      return 0.0f;
    }
  }

  // Proportional current
  float pCurrent = kpAmpsPerDeg * absErr;

  // Soft minimum current assist — overcomes stiction
  float blendZone = deadbandDeg * 0.5f;
  float blend     = smoothstep((absErr - deadbandDeg) / max(blendZone, 0.01f));
  float assistA   = minCurrentA * blend;

  float mag = max(pCurrent, assistA);
  mag = clampf(mag, 0.0f, maxCurrentA);

  return (err >= 0.0f) ? mag : -mag;
}

// ---------------- Serial command parser ----------------
bool applyAllJointTargets(float a, float b, float c) {
  if (targetViolatesLinkClearance(a, b, c)) {
    Serial.println(F("ERR target link_clearance_lt_15deg"));
    return false;
  }

  directHomeMode = false;
  startupHomeActive = false;
  // Do not clamp here. The Python simulation/IK layer is responsible for
  // workspace limits. The firmware only converts the requested joint angle
  // into an absolute encoder target and takes the shortest encoder path.
  targetDeg[0] = a;
  targetDeg[1] = b;
  targetDeg[2] = c;

  for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
  lastCommandMs = millis();
  return true;
}

uint8_t xorChecksum(const char *body) {
  uint8_t cs = 0;
  while (*body) {
    cs ^= (uint8_t)(*body);
    body++;
  }
  return cs;
}

bool parseSimPacket(char *line) {
  if (line[0] != '$') return false;

  char *star = strchr(line, '*');
  if (star == nullptr) {
    Serial.println(F("ERR S missing_checksum"));
    return true;
  }

  *star = '\0';
  const char *body = line + 1;
  uint8_t got = (uint8_t)strtoul(star + 1, nullptr, 16);
  uint8_t calc = xorChecksum(body);

  if (got != calc) {
    Serial.print(F("ERR S checksum got="));
    Serial.print(got, HEX);
    Serial.print(F(" calc="));
    Serial.println(calc, HEX);
    return true;
  }

  float a = 0.0f, b = 0.0f, c = 0.0f;
  int en = 0;
  if (sscanf(body, "S,%f,%f,%f,%d", &a, &b, &c, &en) != 4) {
    Serial.println(F("ERR S format"));
    return true;
  }

  if (en == 0) {
    enabled = false;
    releaseAll();
    for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
    lastCommandMs = millis();
    Serial.println(F("ACK S DISABLE"));
    return true;
  }

  if (applyAllJointTargets(a, b, c)) {
    enabled = true;
    Serial.print(F("ACK S "));
    Serial.print(targetDeg[0], 2); Serial.print(F(" "));
    Serial.print(targetDeg[1], 2); Serial.print(F(" "));
    Serial.println(targetDeg[2], 2);
  }
  return true;
}

void holdCurrentPosition() {
  directHomeMode = false;
  startupHomeActive = false;
  bool ok = true;
  float nowDeg[3];
  for (uint8_t j = 0; j < N_JOINTS; j++) {
    nowDeg[j] = readEncoderDeg(j);
    if (isnan(nowDeg[j])) ok = false;
  }

  if (!ok) { Serial.println(F("ERR HOLD encoder_not_live")); return; }

  targetDeg[0] = nowDeg[0];
  targetDeg[1] = nowDeg[1];
  targetDeg[2] = nowDeg[2];
  for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
  enabled = true;
  lastCommandMs = millis();

  Serial.print(F("ACK HOLD "));
  Serial.print(targetDeg[0], 2); Serial.print(F(" "));
  Serial.print(targetDeg[1], 2); Serial.print(F(" "));
  Serial.println(targetDeg[2], 2);
}

void commandHomePosition() {
  startupHomeActive = false;
  directHomeMode = true;
  targetDeg[0] = 0.0f;
  targetDeg[1] = 0.0f;
  targetDeg[2] = 0.0f;
  for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
  enabled = true;
  lastCommandMs = millis();
  Serial.println(F("ACK GOHOME"));
}

// ---------------- Serial command parser ----------------
void parseLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (strlen(line) == 0) return;

  if (parseSimPacket(line)) return;

  if (strcmp(line, "HB") == 0) {
    lastCommandMs = millis();
    return;
  }

  if (strcmp(line, "HOME") == 0 || strcmp(line, "GOHOME") == 0) {
    commandHomePosition();
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    directHomeMode = false;
    startupHomeActive = false;
    enabled = false;
    releaseAll();
    for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
    lastCommandMs = millis();
    Serial.println(F("ACK STOP"));
    return;
  }

  if (strcmp(line, "HOLD") == 0) {
    holdCurrentPosition();
    return;
  }

  if (strcmp(line, "ENABLE") == 0) {
    enabled = true;
    lastCommandMs = millis();
    Serial.println(F("ACK ENABLE"));
    return;
  }

  if (strcmp(line, "DISABLE") == 0) {
    directHomeMode = false;
    startupHomeActive = false;
    enabled = false;
    releaseAll();
    for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = false;
    Serial.println(F("ACK DISABLE"));
    return;
  }

  // MOVEALL <j1> <j2> <j3>
  if (strncmp(line, "MOVEALL", 7) == 0) {
    float a, b, c;
    if (sscanf(line, "MOVEALL %f %f %f", &a, &b, &c) == 3) {
      if (applyAllJointTargets(a, b, c)) {
        Serial.print(F("ACK MOVEALL "));
        Serial.print(targetDeg[0], 2); Serial.print(F(" "));
        Serial.print(targetDeg[1], 2); Serial.print(F(" "));
        Serial.println(targetDeg[2], 2);
      }
    } else {
      Serial.println(F("ERR MOVEALL use MOVEALL <j1> <j2> <j3>"));
    }
    return;
  }

  // MOVE <j> <deg>  — single joint, others unchanged
  if (strncmp(line, "MOVE", 4) == 0) {
    int j = 0; float deg = 0.0f;
    if (sscanf(line, "MOVE %d %f", &j, &deg) == 2 && j >= 1 && j <= 3) {
      float cand[3] = {targetDeg[0], targetDeg[1], targetDeg[2]};
      cand[j-1] = deg;
      if (targetViolatesLinkClearance(cand[0], cand[1], cand[2])) {
        Serial.println(F("ERR MOVE link_clearance_lt_15deg"));
        return;
      }
      directHomeMode = false;
      startupHomeActive = false;
      targetDeg[j-1] = deg;
      settled[j-1]   = false;
      lastCommandMs  = millis();
      Serial.print(F("ACK MOVE J")); Serial.print(j);
      Serial.print(F(" ")); Serial.println(targetDeg[j-1], 2);
    } else {
      Serial.println(F("ERR MOVE use MOVE <1-3> <deg>"));
    }
    return;
  }

  // CONFIG <kp> <min_a> <max_a> <deadband>
  if (strncmp(line, "CONFIG", 6) == 0) {
    float a, b, c, d;
    if (sscanf(line, "CONFIG %f %f %f %f", &a, &b, &c, &d) == 4) {
      kpAmpsPerDeg = clampf(a, 0.01f, 5.0f);
      minCurrentA  = clampf(b, 0.0f,  20.0f);
      maxCurrentA  = clampf(c, 0.0f,  20.0f);
      deadbandDeg  = clampf(d, 0.1f,  10.0f);
      lastCommandMs = millis();
      Serial.print(F("ACK CONFIG kp="));   Serial.print(kpAmpsPerDeg, 3);
      Serial.print(F(" minA="));           Serial.print(minCurrentA, 1);
      Serial.print(F(" maxA="));           Serial.print(maxCurrentA, 1);
      Serial.print(F(" deadband="));       Serial.println(deadbandDeg, 2);
    } else {
      Serial.println(F("ERR CONFIG use CONFIG <kp> <minA> <maxA> <deadband>"));
    }
    return;
  }

  Serial.print(F("ERR UNKNOWN ")); Serial.println(line);
}

void handleSerialLines() {
  static char buf[128];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[idx] = '\0';
      parseLine(buf);
      idx = 0;
      continue;
    }
    if (idx < sizeof(buf) - 1) buf[idx++] = ch;
    else { idx = 0; Serial.println(F("ERR line_too_long")); }
  }
}

// ---------------- Telemetry ----------------
void printTelemetry() {
  float rawEncDeg[3];
  bool rawOk[3];
  for (uint8_t e = 0; e < 3; e++) {
    rawEncDeg[e] = readRawEncoderDegByEnc(e);
    rawOk[e] = !isnan(rawEncDeg[e]);
  }

  Serial.print(F("T "));
  Serial.print(enabled ? F("EN ") : F("DIS "));
  Serial.print(directHomeMode ? F("DIRECT_HOME") : F("FIXEDHOME"));

  // Raw encoder order. These correspond directly to physical encoder pins.
  Serial.print(F(" | ENC_RAW"));
  for (uint8_t e = 0; e < 3; e++) {
    Serial.print(F(" E")); Serial.print(e + 1);
    Serial.print(F("="));
    if (rawOk[e]) Serial.print(rawEncDeg[e], 2);
    else          Serial.print(F("NA"));
  }

  // Joint order. These are the encoder readings matched to J1/M1, J2/M2, J3/M3.
  // Use these values directly for JOINT_HOME_ENCODER_DEG when the robot is physically home.
  Serial.print(F(" | ENC_JOINT"));
  for (uint8_t j = 0; j < N_JOINTS; j++) {
    uint8_t e = JOINT_TO_ENC[j];
    Serial.print(F(" J")); Serial.print(j + 1);
    Serial.print(F("="));
    if (rawOk[e]) Serial.print(rawEncDeg[e], 2);
    else          Serial.print(F("NA"));
  }

  for (uint8_t j = 0; j < N_JOINTS; j++) {
    Serial.print(F(" | J")); Serial.print(j+1);
    Serial.print(F(" tgt=")); Serial.print(targetDeg[j], 1);
    Serial.print(F(" meas="));
    if (measuredOk[j]) Serial.print(measuredDeg[j], 1);
    else                Serial.print(F("NA"));
    Serial.print(F(" err=")); Serial.print(errDeg[j], 1);
    Serial.print(F(" A="));   Serial.print(cmdCurrentA[j], 2);
    Serial.print(settled[j] ? F(" S") : F(" -"));
  }
  Serial.println();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println(F("=== SPM Three-Joint Current Control ==="));
  Serial.println(F("Commands: ENABLE, DISABLE, STOP, HOLD, GOHOME"));
  Serial.println(F("          MOVEALL <j1> <j2> <j3>"));
  Serial.println(F("          MOVE <1-3> <deg>"));
  Serial.println(F("          CONFIG <kp> <minA> <maxA> <deadband>"));
  Serial.println(F("          HB"));
  Serial.println(F("          $S,<j1>,<j2>,<j3>,<enable>*XX from spm_angle_comm.py"));
  Serial.println(F("Defaults: kp=0.30 minA=7.0 maxA=12.0 deadband=2.0"));
  Serial.print(F("Fixed joint home encoder deg J1/J2/J3 = "));
  Serial.print(JOINT_HOME_ENCODER_DEG[0], 2); Serial.print(F(" / "));
  Serial.print(JOINT_HOME_ENCODER_DEG[1], 2); Serial.print(F(" / "));
  Serial.println(JOINT_HOME_ENCODER_DEG[2], 2);
  Serial.println(F("Mapping: J1->E3 pin4, J2->E2 pin3, J3->E1 pin2"));
  Serial.println();

  for (uint8_t i = 0; i < 3; i++) pinMode(ENC_PIN[i], INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_PIN[0]), encISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN[1]), encISR<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN[2]), encISR<2>, CHANGE);

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    VESC_PORT[i]->begin(VESC_BAUD);
    VESC[i].setSerialPort(VESC_PORT[i]);
    VESC[i].setCurrent(0.0f);
  }

  loadFixedHome();
  enabled = ENABLE_AT_STARTUP;
  lastCommandMs = millis();
  startupHomeActive = enabled;
  directHomeMode = enabled;
  startupHomeStartMs = millis();
  startupHomeSettledSinceMs = 0;
  Serial.println(enabled ? F("ACK STARTUP DIRECT_ENCODER_HOME") : F("ACK STARTUP READY"));
}


// ---------------- Loop ----------------
void loop() {
  handleSerialLines();

  uint32_t now = millis();

  // ---- 100Hz control loop ----
  static uint32_t lastControlMs = 0;
  if (now - lastControlMs >= (1000 / CONTROL_HZ)) {
    lastControlMs = now;

    // Watchdog — no HB or command in timeout → disable.
    // Startup direct-home gets a longer one-shot window and stops early once
    // every joint has remained inside the deadband for STARTUP_HOME_SETTLED_MS.
    if (!startupHomeActive && enabled && (now - lastCommandMs) > COMMAND_TIMEOUT_MS) {
      enabled = false;
      directHomeMode = false;
      releaseAll();
      Serial.println(F("WARN timeout -- disabled"));
    }

    bool allSettledNow = true;

    // Update all joints
    for (uint8_t j = 0; j < N_JOINTS; j++) {
      float raw = readEncoderDeg(j);
      measuredOk[j]  = !isnan(raw);
      measuredDeg[j] = measuredOk[j] ? raw : measuredDeg[j];

      if (enabled && measuredOk[j]) {
        if (directHomeMode) {
          uint8_t enc = JOINT_TO_ENC[j];
          float absDeg = readRawEncoderDegByEnc(enc);
          measuredOk[j] = !isnan(absDeg);
          if (measuredOk[j]) {
            // Direct startup/GOHOME behavior: the target is the fixed raw
            // encoder home angle, independent of joint-angle sign convention.
            errDeg[j] = shortestAngleErrorDeg(homeEncoderDegForJoint(j), absDeg);
          }
        } else {
          // Normal simulation/MOVE behavior. This still resolves through the
          // fixed encoder home, but targetDeg may be nonzero.
          uint8_t enc = JOINT_TO_ENC[j];
          float absDeg = readRawEncoderDegByEnc(enc);
          measuredOk[j] = !isnan(absDeg);
          if (measuredOk[j]) {
            float targetAbs = jointTargetToRawEncoderDeg(j);
            // Keep the controller error in RAW ENCODER coordinates for both
            // GOHOME and normal Python/MOVE commands. This avoids direction
            // reversals caused by mixing joint-coordinate error with
            // encoder-coordinate motor polarity, especially near 0/360 wrap.
            errDeg[j] = shortestAngleErrorDeg(targetAbs, absDeg);
          }
        }

        if (measuredOk[j]) {
          cmdCurrentA[j] = (float)MOTOR_SIGN[j] * computeCurrentCmd(errDeg[j], j);
          VESC[j].setCurrent(cmdCurrentA[j]);
          if (fabsf(errDeg[j]) > deadbandDeg) allSettledNow = false;
        } else {
          cmdCurrentA[j] = 0.0f;
          VESC[j].setCurrent(0.0f);
          allSettledNow = false;
        }
      } else {
        errDeg[j]      = 0.0f;
        cmdCurrentA[j] = 0.0f;
        VESC[j].setCurrent(0.0f);
        allSettledNow = false;
      }
    }

    if (startupHomeActive) {
      if (allSettledNow) {
        if (startupHomeSettledSinceMs == 0) startupHomeSettledSinceMs = now;
        if ((now - startupHomeSettledSinceMs) >= STARTUP_HOME_SETTLED_MS) {
          startupHomeActive = false;
          directHomeMode = false;
          enabled = false;
          targetDeg[0] = 0.0f;
          targetDeg[1] = 0.0f;
          targetDeg[2] = 0.0f;
          for (uint8_t j = 0; j < N_JOINTS; j++) settled[j] = true;
          releaseAll();
          Serial.println(F("ACK startup_home_settled -- neutral disabled"));
        }
      } else {
        startupHomeSettledSinceMs = 0;
      }

      if (startupHomeActive && (now - startupHomeStartMs) > STARTUP_HOME_RUN_MS) {
        startupHomeActive = false;
        directHomeMode = false;
        enabled = false;
        targetDeg[0] = 0.0f;
        targetDeg[1] = 0.0f;
        targetDeg[2] = 0.0f;
        releaseAll();
        Serial.println(F("WARN startup_home_timeout -- neutral disabled"));
      }
    }
  }

  // ---- 10Hz telemetry ----
  static uint32_t lastTelemMs = 0;
  if (now - lastTelemMs >= (1000 / TELEM_HZ)) {
    lastTelemMs = now;
    printTelemetry();
  }
}