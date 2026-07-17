#define ARDUINO_USB_CDC_ON_BOOT 1
// firebot-s3 v2.3 — esp32-s3 n16r8 (optimized with watchdog) stuff, i think this will do this, ayy dont mess this upp

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <MPU6050.h>

// watchdog configuration stuff, i think this will do this, ayy dont mess this upp
#define WDT_TIMEOUT_SEC 5

// pin map stuff, i think this will do this, ayy dont mess this upp
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint8_t TMP117_ADDR = 0x48;
constexpr uint8_t MPU6050_ADDR = 0x68;

constexpr uint8_t MQ2_PIN = 6;
constexpr uint8_t MQ5_PIN = 7;
constexpr uint8_t FLAME_D0 = 4;

constexpr uint8_t ENC_LEFT_PIN = 13;
constexpr uint8_t ENC_RIGHT_PIN = 14;

constexpr uint8_t SERVO_SCAN_PIN = 12;
constexpr uint8_t SERVO_TURRET_PIN = 11;
constexpr uint8_t SERVO_TILT_PIN = 18;

constexpr uint8_t PUMP_PIN = 5;

constexpr uint8_t AIN1 = 21;
constexpr uint8_t AIN2 = 47;
constexpr uint8_t ENA = 10;

constexpr uint8_t BIN1 = 15;
constexpr uint8_t BIN2 = 17;
constexpr uint8_t ENB = 16;

constexpr uint8_t LED_PIN = 48;
constexpr uint8_t LED_COUNT = 1;

// config stuff, i think this will do this, ayy dont mess this upp
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

constexpr uint16_t PWM_FREQ_HZ = 1000;
constexpr uint32_t PWM_PERIOD_US = 1000000UL / PWM_FREQ_HZ;
constexpr uint8_t PWM_RESOLUTION = 100;

constexpr float WHEEL_CIRC = 0.2042f;
constexpr float PULSES_PER_REV = 5.0f;
constexpr float WHEEL_BASE = 0.15f;
// REMOVED: DEG_TO_RAD and RAD_TO_DEG (already defined in Arduino.h)

// forward declarations stuff, i think this will do this, ayy dont mess this upp
struct ServoState;
struct ScanResult;

// state machine stuff, i think this will do this, ayy dont mess this upp
enum SystemState : uint8_t {
  STATE_BOOT_INIT,
  STATE_SELF_TEST,
  STATE_WAITING_PI,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_ERROR
};

struct SystemFlags {
  uint8_t piReady : 1;
  uint8_t manualOverride : 1;
  uint8_t calibrationComplete : 1;
  uint8_t mpuCalibrated : 1;
  uint8_t mqCalibrated : 1;
  uint8_t i2cDevicesOK : 1;
  uint8_t scanningEnabled : 1;
  uint8_t samplingEndpoint : 1;
  uint8_t pumpActive : 1;
};

SystemState currentState = STATE_BOOT_INIT;
SystemFlags flags = {0};

unsigned long bootStartTime = 0;
unsigned long lastPingTime = 0;
constexpr uint16_t PING_INTERVAL_MS = 1000;
constexpr uint32_t HANDSHAKE_TIMEOUT_MS = 25000;

// led status stuff, i think this will do this, ayy dont mess this upp
Adafruit_NeoPixel statusLED(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
constexpr uint8_t LED_BRIGHTNESS = 25;

const uint32_t COLOR_BLUE = Adafruit_NeoPixel::Color(0, 0, LED_BRIGHTNESS);
const uint32_t COLOR_YELLOW = Adafruit_NeoPixel::Color(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
const uint32_t COLOR_GREEN = Adafruit_NeoPixel::Color(0, LED_BRIGHTNESS, 0);
const uint32_t COLOR_RED = Adafruit_NeoPixel::Color(LED_BRIGHTNESS, 0, 0);
const uint32_t COLOR_OFF = Adafruit_NeoPixel::Color(0, 0, 0);

inline void setLEDColor(uint32_t color) { 
  statusLED.setPixelColor(0, color); 
  statusLED.show(); 
}

void flashLED(uint32_t color, uint8_t times, uint16_t delayMs) {
  for (uint8_t i = 0; i < times; i++) { 
    setLEDColor(color); 
    delay(delayMs); 
    setLEDColor(COLOR_OFF); 
    delay(delayMs); 
  }
}

// servos (esp32servo) stuff, i think this will do this, ayy dont mess this upp
Servo scanServo;
Servo turretServo;
Servo tiltServo;

struct ServoState {
  int16_t current;
  int16_t target;
};

ServoState scanAngle = {90, 90};
ServoState turretAngle = {90, 90};
ServoState tiltAngle = {90, 90};

inline int16_t clampAngle(int16_t angle) {
  return (angle < 0) ? 0 : ((angle > 180) ? 180 : angle);
}

void servoWriteAngle(Servo &sv, int16_t angle) {
  angle = clampAngle(angle);
  if (currentState != STATE_READY && currentState != STATE_CALIBRATING && !flags.manualOverride) {
    angle = 90;
  }
  sv.write(angle);
}

void smoothServoMove(Servo &sv, ServoState &state, int8_t step = 2) {
  if (state.current < state.target) {
    state.current += min((int16_t)step, (int16_t)(state.target - state.current));
  } else if (state.current > state.target) {
    state.current -= min((int16_t)step, (int16_t)(state.current - state.target));
  }
  servoWriteAngle(sv, state.current);
}

void setServoImmediate(Servo &sv, ServoState &state, int16_t angle) { 
  state.current = state.target = clampAngle(angle); 
  sv.write(state.current); 
}

void centerAllServos() {
  setServoImmediate(scanServo, scanAngle, 90);
  setServoImmediate(turretServo, turretAngle, 90);
  setServoImmediate(tiltServo, tiltAngle, 90);
}

void updateServos() {
  smoothServoMove(scanServo, scanAngle, 3);
  smoothServoMove(turretServo, turretAngle, 3);
  smoothServoMove(tiltServo, tiltAngle, 3);
}

// software pwm (motors) stuff, i think this will do this, ayy dont mess this upp
volatile int8_t dutyA = 0;
volatile int8_t dutyB = 0;

static bool lastEnaState = false;
static bool lastEnbState = false;

inline void setMotorPinsA(bool forward) {
  digitalWrite(AIN1, forward);
  digitalWrite(AIN2, !forward);
}

inline void setMotorPinsB(bool forward) {
  digitalWrite(BIN1, forward);
  digitalWrite(BIN2, !forward);
}

void applyMotorSpeeds() {
  if (currentState != STATE_READY && !flags.manualOverride) return;
  setMotorPinsA(dutyA >= 0);
  setMotorPinsB(dutyB >= 0);
}

void updateSoftwarePWM() {
  static uint32_t periodStart = 0;
  uint32_t now = micros();
  uint32_t elapsed = now - periodStart;
  
  if (elapsed >= PWM_PERIOD_US) {
    periodStart = now;
    elapsed = 0;
  }

  uint8_t absDutyA = (dutyA >= 0) ? dutyA : -dutyA;
  uint8_t absDutyB = (dutyB >= 0) ? dutyB : -dutyB;
  
  if (absDutyA > PWM_RESOLUTION) absDutyA = PWM_RESOLUTION;
  if (absDutyB > PWM_RESOLUTION) absDutyB = PWM_RESOLUTION;

  uint32_t highTimeA = (PWM_PERIOD_US * absDutyA) / PWM_RESOLUTION;
  uint32_t highTimeB = (PWM_PERIOD_US * absDutyB) / PWM_RESOLUTION;

  bool enaState = (elapsed < highTimeA);
  bool enbState = (elapsed < highTimeB);
  
  if (enaState != lastEnaState) {
    digitalWrite(ENA, enaState);
    lastEnaState = enaState;
  }
  if (enbState != lastEnbState) {
    digitalWrite(ENB, enbState);
    lastEnbState = enbState;
  }
}

int8_t leftMotorSpeed = 0;
int8_t rightMotorSpeed = 0;

inline int8_t clampSpeed(int val) {
  return (val < -100) ? -100 : ((val > 100) ? 100 : val);
}

void setLeftMotor(int pct) { 
  leftMotorSpeed = clampSpeed(pct); 
  dutyA = leftMotorSpeed; 
}

void setRightMotor(int pct) { 
  rightMotorSpeed = clampSpeed(pct); 
  dutyB = rightMotorSpeed; 
}

void motorStop() {
  setLeftMotor(0); 
  setRightMotor(0);
  digitalWrite(ENA, LOW); 
  digitalWrite(ENB, LOW);
  digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); 
  digitalWrite(BIN2, LOW);
  lastEnaState = lastEnbState = false;
}

inline void motorsOff() { motorStop(); }

// sensors / calibration stuff, i think this will do this, ayy dont mess this upp
int16_t mq2Baseline = 0;
int16_t mq5Baseline = 0;
float ambientTemp = 25.0f;

inline float normalizeADC(int raw) { 
  return (raw < 0) ? 0.0f : ((raw > 4095) ? 1.0f : (raw * 0.000244140625f));
}

inline float tempDeltaScore(float t) { 
  float delta = (t - ambientTemp) * 0.05f;
  return (delta < 0.0f) ? 0.0f : ((delta > 1.0f) ? 1.0f : delta);
}

bool checkI2CDevice(uint8_t address) { 
  Wire.beginTransmission(address); 
  return (Wire.endTransmission() == 0); 
}

float readTMP117() {
  static float lastValid = 25.0f;
  static uint32_t lastRead = 0;
  
  uint32_t now = millis();
  if (now - lastRead < 250) return lastValid;
  
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(TMP117_ADDR, (uint8_t)2) == 2) {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    float t = raw * 0.0078125f;
    if (t > -40.0f && t < 125.0f) { 
      lastValid = t; 
      lastRead = now; 
    }
  }
  return lastValid;
}

void calibrateMQBaselines() {
  Serial.println(F("{\"type\":\"calibration\",\"sensor\":\"mq\",\"status\":\"starting\"}"));
  int32_t s2 = 0, s5 = 0;
  constexpr uint8_t samples = 30;
  
  for (uint8_t i = 0; i < samples; i++) { 
    esp_task_wdt_reset();
    s2 += analogRead(MQ2_PIN); 
    s5 += analogRead(MQ5_PIN); 
    delay(40); 
  }
  
  mq2Baseline = s2 / samples; 
  mq5Baseline = s5 / samples; 
  flags.mqCalibrated = 1;
  
  Serial.printf("{\"type\":\"calibration\",\"sensor\":\"mq\",\"status\":\"complete\",\"mq2_baseline\":%d,\"mq5_baseline\":%d}\n", 
                mq2Baseline, mq5Baseline);
}

// encoders / odometry stuff, i think this will do this, ayy dont mess this upp
volatile uint32_t leftPulses = 0;
volatile uint32_t rightPulses = 0;
volatile uint32_t lastLeftMicros = 0;
volatile uint32_t lastRightMicros = 0;
constexpr uint32_t DEBOUNCE_US = 1200;

void IRAM_ATTR IR_left_ISR() { 
  uint32_t now = micros(); 
  if (now - lastLeftMicros >= DEBOUNCE_US) { 
    leftPulses++; 
    lastLeftMicros = now; 
  } 
}

void IRAM_ATTR IR_right_ISR() { 
  uint32_t now = micros(); 
  if (now - lastRightMicros >= DEBOUNCE_US) { 
    rightPulses++; 
    lastRightMicros = now; 
  } 
}

uint32_t lastRPMcalc = 0;
float leftRPM = 0.0f, rightRPM = 0.0f;
uint32_t lastLeftPulseCount = 0;
uint32_t lastRightPulseCount = 0;

// MPU
MPU6050 mpu;
int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
int16_t axOffset = 0, ayOffset = 0, azOffset = 0, gzOffset = 0;
float gz_deg_s = 0.0f;
float ax_g = 0.0f, ay_g = 0.0f;
uint32_t lastMPUmicros = 0;

constexpr float ACCEL_NOISE_THRESHOLD = 0.05f;
constexpr float GYRO_NOISE_THRESHOLD = 2.0f;
constexpr float ACCEL_SCALE = 1.0f / 16384.0f;
constexpr float GYRO_SCALE = 1.0f / 131.0f;

void calibrateMPUOffsets() {
  Serial.println(F("{\"type\":\"calibration\",\"sensor\":\"mpu\",\"status\":\"starting\"}"));
  int32_t axSum = 0, aySum = 0, azSum = 0, gzSum = 0;
  constexpr uint16_t samples = 300;
  
  for (uint16_t i = 0; i < samples; i++) { 
    esp_task_wdt_reset();
    mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw); 
    axSum += axRaw; 
    aySum += ayRaw; 
    azSum += azRaw; 
    gzSum += gzRaw; 
    delay(5); 
  }
  
  axOffset = axSum / samples; 
  ayOffset = aySum / samples; 
  azOffset = azSum / samples; 
  gzOffset = gzSum / samples;
  flags.mpuCalibrated = 1;
  
  Serial.printf("{\"type\":\"calibration\",\"sensor\":\"mpu\",\"status\":\"complete\",\"offsets\":{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gz\":%d}}\n", 
                axOffset, ayOffset, azOffset, gzOffset);
}

float posX = 0.0f, posY = 0.0f, yawDeg = 0.0f;

void updateMPU() {
  uint32_t now = micros();
  float dt = (lastMPUmicros == 0) ? 0.01f : (now - lastMPUmicros) * 0.000001f;
  lastMPUmicros = now;
  
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission(false) != 0) return;
  
  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);
  
  ax_g = (axRaw - axOffset) * ACCEL_SCALE;
  ay_g = (ayRaw - ayOffset) * ACCEL_SCALE;
  gz_deg_s = (gzRaw - gzOffset) * GYRO_SCALE;
  yawDeg += gz_deg_s * dt;
}

bool isRobotMoving() {
  float accelMagSq = ax_g * ax_g + ay_g * ay_g;
  return (accelMagSq > ACCEL_NOISE_THRESHOLD * ACCEL_NOISE_THRESHOLD) || 
         (gz_deg_s > GYRO_NOISE_THRESHOLD || gz_deg_s < -GYRO_NOISE_THRESHOLD);
}

void computeRPM() {
  uint32_t now = millis();
  if (now - lastRPMcalc < 1000) return;
  
  uint32_t leftDelta = leftPulses - lastLeftPulseCount;
  uint32_t rightDelta = rightPulses - lastRightPulseCount;
  
  if (!isRobotMoving() && (leftDelta > 0 || rightDelta > 0)) {
    Serial.printf("{\"type\":\"noise_reject\",\"left\":%lu,\"right\":%lu}\n", leftDelta, rightDelta);
    leftDelta = rightDelta = 0;
  }
  
  constexpr float RPM_FACTOR = 60.0f / PULSES_PER_REV;
  leftRPM = leftDelta * RPM_FACTOR;
  rightRPM = rightDelta * RPM_FACTOR;
  
  lastLeftPulseCount = leftPulses;
  lastRightPulseCount = rightPulses;
  lastRPMcalc = now;
}

void updateOdometry() {
  static uint32_t lastOdomMs = 0;
  static uint32_t lastSendMs = 0;
  
  uint32_t now = millis();
  if (now - lastOdomMs < 40) return;
  
  float dt = (now - lastOdomMs) * 0.001f;
  lastOdomMs = now;
  
  constexpr float VEL_FACTOR = WHEEL_CIRC / 60.0f;
  float Lv = leftRPM * VEL_FACTOR;
  float Rv = rightRPM * VEL_FACTOR;
  float v = (Lv + Rv) * 0.5f;
  float dYaw = (Rv - Lv) / WHEEL_BASE;
  
  float yawRad = yawDeg * DEG_TO_RAD;
  posX += v * cosf(yawRad) * dt;
  posY += v * sinf(yawRad) * dt;
  yawDeg += dYaw * dt * RAD_TO_DEG * 0.1f;
  
  if (now - lastSendMs > 500) {
    lastSendMs = now;
    Serial.printf("{\"type\":\"odom\",\"t\":%lu,\"x\":%.3f,\"y\":%.3f,\"yaw\":%.2f,\"rpm_l\":%.2f,\"rpm_r\":%.2f}\n",
                  now, posX, posY, yawDeg, leftRPM, rightRPM);
  }
}

// scanning stuff, i think this will do this, ayy dont mess this upp
constexpr int16_t SCAN_MIN = 20;
constexpr int16_t SCAN_MAX = 160;
constexpr uint8_t ENDPOINT_SAMPLES = 16;
constexpr int16_t NOISE_FLOOR = 40;
constexpr int8_t SWEEP_STEP = 3;

enum ScanMode : uint8_t { SCAN_FIRE, SCAN_DIRECTION };
enum ScanState : uint8_t { 
  MOVING_TO_LEFT, AT_LEFT, MOVING_TO_CENTER1, AT_CENTER1, 
  MOVING_TO_RIGHT, AT_RIGHT, MOVING_TO_CENTER2, AT_CENTER2, SCAN_COMPLETE 
};

ScanMode scanMode = SCAN_FIRE;
ScanState scanState = MOVING_TO_LEFT;

int16_t endpointAngle = 0;
uint8_t samplesTaken = 0;
double mq2Acc = 0, mq5Acc = 0, tempAcc = 0;
uint8_t flameCount = 0;

struct ScanResult {
  float mq2_avg;
  float mq5_avg;
  float temp_avg;
  float score;
  int16_t angle;
  uint8_t flame_hits;
  char dominant[8];
};

ScanResult leftScan = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, ""};
ScanResult rightScan = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, ""};
bool leftScanned = false, rightScanned = false;

// Forward declarations
void startPump(int16_t angle);

void sendScanSampleJSON(int16_t angle, int m2, int m5, float t, int flameD, uint8_t sampleNo) {
  Serial.printf("{\"type\":\"scan_sample\",\"t\":%lu,\"angle\":%d,\"mq2\":%d,\"mq5\":%d,\"temp\":%.2f,\"flame\":%d,\"sample\":%d}\n",
                millis(), angle, m2, m5, t, flameD, sampleNo);
}

void sendScanResultJSON(ScanResult &r, const char *side) {
  Serial.printf("{\"type\":\"scan_result\",\"t\":%lu,\"side\":\"%s\",\"angle\":%d,\"mq2_avg\":%.1f,\"mq5_avg\":%.1f,\"temp_avg\":%.2f,\"flame_hits\":%d,\"score\":%.3f,\"dominant\":\"%s\"}\n",
                millis(), side, r.angle, r.mq2_avg, r.mq5_avg, r.temp_avg, r.flame_hits, r.score, r.dominant);
}

void sendScanCompleteJSON(const char *direction, const char *reason, ScanResult *best) {
  Serial.printf("{\"type\":\"scan_complete\",\"t\":%lu,\"direction\":\"%s\",\"reason\":\"%s\"", millis(), direction, reason);
  if (best) {
    Serial.printf(",\"angle\":%d,\"score\":%.3f,\"mq2_avg\":%.1f,\"mq5_avg\":%.1f,\"temp_avg\":%.2f,\"flame_hits\":%d",
                  best->angle, best->score, best->mq2_avg, best->mq5_avg, best->temp_avg, best->flame_hits);
  }
  Serial.printf(",\"left\":{\"angle\":%d,\"score\":%.3f},\"right\":{\"angle\":%d,\"score\":%.3f}}\n", 
                leftScan.angle, leftScan.score, rightScan.angle, rightScan.score);
}

float computeScore(ScanResult &s) {
  float smoke = normalizeADC((int)s.mq2_avg);
  float gas = normalizeADC((int)s.mq5_avg);
  float tempS = tempDeltaScore(s.temp_avg);
  
  if (s.flame_hits >= 2) { 
    strcpy(s.dominant, "flame"); 
    s.score = 1.0f + 0.4f * smoke + 0.2f * tempS + 0.05f * gas; 
  } else if (smoke > 0.4f) { 
    strcpy(s.dominant, "smoke"); 
    s.score = 0.7f * smoke + 0.3f * tempS; 
  } else if (tempS > 0.2f) { 
    strcpy(s.dominant, "heat");  
    s.score = 0.5f * tempS + 0.3f * smoke + 0.2f * gas; 
  } else {
    strcpy(s.dominant, "none"); 
    s.score = 0.3f * smoke + 0.3f * gas + 0.4f * tempS;
  }
  return s.score;
}

void startSamplingEndpoint(int16_t angle) { 
  flags.samplingEndpoint = 1; 
  endpointAngle = angle; 
  scanAngle.target = angle; 
  samplesTaken = 0; 
  mq2Acc = mq5Acc = tempAcc = 0; 
  flameCount = 0; 
}

void finishSamplingEndpoint() {
  if (samplesTaken == 0) { 
    flags.samplingEndpoint = 0; 
    return; 
  }
  
  ScanResult r;
  r.angle = endpointAngle;
  r.mq2_avg = mq2Acc / samplesTaken; 
  r.mq5_avg = mq5Acc / samplesTaken; 
  r.temp_avg = tempAcc / samplesTaken; 
  r.flame_hits = flameCount;
  r.score = 0.0f;
  strcpy(r.dominant, "");
  computeScore(r);
  
  if (endpointAngle == SCAN_MIN) { 
    leftScan = r; 
    leftScanned = true; 
    sendScanResultJSON(leftScan, "left"); 
  } else if (endpointAngle == SCAN_MAX) { 
    rightScan = r; 
    rightScanned = true; 
    sendScanResultJSON(rightScan, "right"); 
  }
  flags.samplingEndpoint = 0;
}

void evaluateAndPublishDirection() {
  if (!leftScanned && !rightScanned) { 
    sendScanCompleteJSON("none", "no_data", nullptr); 
    return; 
  }
  
  ScanResult *best = nullptr; 
  const char *dir = "none"; 
  const char *reason = "none";
  
  if (leftScanned && rightScanned) {
    if (leftScan.score > rightScan.score + 0.02f) { 
      best = &leftScan; dir = "left"; reason = "higher_score_left"; 
    } else if (rightScan.score > leftScan.score + 0.02f) { 
      best = &rightScan; dir = "right"; reason = "higher_score_right"; 
    } else { 
      float la = (leftScan.mq2_avg + leftScan.mq5_avg) * 0.5f; 
      float ra = (rightScan.mq2_avg + rightScan.mq5_avg) * 0.5f;
      if (la > ra) { best = &leftScan; dir = "left"; reason = "avg_left"; } 
      else { best = &rightScan; dir = "right"; reason = "avg_right"; } 
    }
  } else if (leftScanned) { 
    best = &leftScan; dir = "left"; reason = "only_left"; 
  } else { 
    best = &rightScan; dir = "right"; reason = "only_right"; 
  }
  
  bool smokeHigh = (best->mq2_avg >= 250.0f);
  bool gasHigh = (best->mq5_avg >= 200.0f);
  bool tempRise = (best->temp_avg - ambientTemp) >= 2.0f;
  bool danger = smokeHigh || gasHigh || tempRise || (best->flame_hits > 0);
  
  if (!danger) {
    sendScanCompleteJSON("none", "weak_signal", best);
  } else {
    sendScanCompleteJSON(dir, reason, best);
    if (best->flame_hits > 0 && !flags.pumpActive) { 
      startPump(best->angle); 
    }
  }
}

void handleScanningTick(uint32_t now) {
  static uint32_t lastSweep = 0;
  static uint32_t lastSample = 0;
  static int8_t _scanDir = 1;
  static uint32_t _lastFlameTick = 0;
  static uint8_t flameConfirmCount = 0;

  if (now - _lastFlameTick >= 100) {
    _lastFlameTick = now;
    if (digitalRead(FLAME_D0) == HIGH) {
      flameConfirmCount++;
      if (flameConfirmCount >= 3) {
        Serial.printf("{\"type\":\"alert\",\"msg\":\"flame_confirmed\",\"angle\":%d}\n", scanAngle.current);
        if (!flags.pumpActive) startPump(scanAngle.current);
        flameConfirmCount = 0;
      }
    } else {
      flameConfirmCount = 0;
    }
  }

  if (!flags.scanningEnabled) return;
  if (currentState != STATE_READY && !flags.manualOverride) return;

  if (!flags.samplingEndpoint && (now - lastSweep >= 40)) {
    lastSweep = now;
    if (scanMode == SCAN_FIRE) {
      if (scanAngle.current >= SCAN_MAX) _scanDir = -1;
      else if (scanAngle.current <= SCAN_MIN) _scanDir = 1;
      
      int16_t newAngle = scanAngle.current + (_scanDir * SWEEP_STEP);
      scanAngle.target = (newAngle < SCAN_MIN) ? SCAN_MIN : ((newAngle > SCAN_MAX) ? SCAN_MAX : newAngle);
      scanAngle.current = scanAngle.target;
    } else {
      switch (scanState) {
        case MOVING_TO_LEFT:
          scanAngle.target = SCAN_MIN;
          if (abs(scanAngle.current - SCAN_MIN) <= 2) { 
            scanState = AT_LEFT; 
            scanAngle.current = SCAN_MIN; 
            startSamplingEndpoint(SCAN_MIN); 
          } else {
            scanAngle.current = max((int16_t)SCAN_MIN, (int16_t)(scanAngle.current - SWEEP_STEP));
          }
          break;
        case MOVING_TO_CENTER1:
          scanAngle.target = 90;
          if (abs(scanAngle.current - 90) <= 2) { 
            scanState = AT_CENTER1; 
            scanAngle.current = 90; 
            startSamplingEndpoint(90); 
          } else {
            scanAngle.current = (scanAngle.current < 90) ? 
              min((int16_t)90, (int16_t)(scanAngle.current + SWEEP_STEP)) : 
              max((int16_t)90, (int16_t)(scanAngle.current - SWEEP_STEP));
          }
          break;
        case MOVING_TO_RIGHT:
          scanAngle.target = SCAN_MAX;
          if (abs(scanAngle.current - SCAN_MAX) <= 2) { 
            scanState = AT_RIGHT; 
            scanAngle.current = SCAN_MAX; 
            startSamplingEndpoint(SCAN_MAX); 
          } else {
            scanAngle.current = min((int16_t)SCAN_MAX, (int16_t)(scanAngle.current + SWEEP_STEP));
          }
          break;
        case MOVING_TO_CENTER2:
          scanAngle.target = 90;
          if (abs(scanAngle.current - 90) <= 2) { 
            scanState = AT_CENTER2; 
            scanAngle.current = 90; 
            startSamplingEndpoint(90); 
          } else {
            scanAngle.current = (scanAngle.current < 90) ? 
              min((int16_t)90, (int16_t)(scanAngle.current + SWEEP_STEP)) : 
              max((int16_t)90, (int16_t)(scanAngle.current - SWEEP_STEP));
          }
          break;
        default: break;
      }
    }
  }

  if (flags.samplingEndpoint && (now - lastSample >= 250) && (samplesTaken < ENDPOINT_SAMPLES)) {
    lastSample = now;
    int raw_m2 = analogRead(MQ2_PIN);
    int raw_m5 = analogRead(MQ5_PIN);
    int m2 = max(0, raw_m2 - mq2Baseline);
    int m5 = max(0, raw_m5 - mq5Baseline);
    if (m2 < NOISE_FLOOR) m2 = 0;
    if (m5 < NOISE_FLOOR) m5 = 0;
    float t = readTMP117();
    int flameD = digitalRead(FLAME_D0);
    
    mq2Acc += m2; 
    mq5Acc += m5; 
    tempAcc += t;
    if (flameD == HIGH) flameCount++;
    samplesTaken++;
    
    sendScanSampleJSON(endpointAngle, m2, m5, t, flameD, samplesTaken);
    if (flameD == HIGH && !flags.pumpActive) startPump(endpointAngle);
    
    if (samplesTaken >= ENDPOINT_SAMPLES) {
      finishSamplingEndpoint();
      if (scanMode == SCAN_DIRECTION) {
        if (scanState == AT_LEFT) scanState = MOVING_TO_CENTER1;
        else if (scanState == AT_CENTER1) scanState = MOVING_TO_RIGHT;
        else if (scanState == AT_RIGHT) scanState = MOVING_TO_CENTER2;
        else if (scanState == AT_CENTER2) { 
          scanState = SCAN_COMPLETE; 
          evaluateAndPublishDirection(); 
          flags.scanningEnabled = 0; 
        }
      }
    }
  }
}

// pump stuff, i think this will do this, ayy dont mess this upp
uint32_t pumpStartTime = 0;
constexpr uint32_t PUMP_DURATION = 2500;

void startPump(int16_t angle) {
  if (currentState != STATE_READY && !flags.manualOverride) return;
  if (flags.pumpActive) return;
  
  flags.pumpActive = 1;
  turretAngle.target = clampAngle(angle);
  digitalWrite(PUMP_PIN, HIGH);
  pumpStartTime = millis();
  Serial.printf("{\"type\":\"pump\",\"status\":\"on\",\"angle\":%d}\n", angle);
}

void updatePump() {
  if (!flags.pumpActive) return;
  if (millis() - pumpStartTime >= PUMP_DURATION) {
    digitalWrite(PUMP_PIN, LOW);
    flags.pumpActive = 0;
    turretAngle.target = 90;
    Serial.println(F("{\"type\":\"pump\",\"status\":\"off\"}"));
  }
}

void pumpOff() { 
  digitalWrite(PUMP_PIN, LOW); 
  flags.pumpActive = 0; 
}

// pi handshake stuff, i think this will do this, ayy dont mess this upp
char incomingBuffer[128];
uint8_t incomingIdx = 0;
uint32_t lastCharTime = 0;

const char* getStateName() {
  static const char* names[] = {"boot_init", "self_test", "waiting_pi", "calibrating", "ready", "error"};
  return (currentState <= STATE_ERROR) ? names[currentState] : "unknown";
}

void sendPing() { 
  Serial.printf("{\"type\":\"ping\",\"t\":%lu,\"state\":\"%s\"}\n", millis(), getStateName()); 
}

void handlePiHandshake() {
  uint32_t now = millis();
  uint32_t elapsed = now - bootStartTime;
  
  if (now - lastPingTime >= PING_INTERVAL_MS) { 
    lastPingTime = now; 
    sendPing(); 
  }
  
  if (!flags.piReady && !flags.manualOverride && elapsed >= HANDSHAKE_TIMEOUT_MS) {
    flags.manualOverride = 1;
    Serial.println(F("{\"type\":\"handshake\",\"status\":\"timeout\",\"msg\":\"Auto-override activated\"}"));
  }
}

void updateStateMachine() {
  switch (currentState) {
    case STATE_BOOT_INIT:
    case STATE_SELF_TEST: 
      break;
      
    case STATE_WAITING_PI:
      setLEDColor(COLOR_BLUE);
      handlePiHandshake();
      if (flags.piReady || flags.manualOverride) {
        currentState = STATE_CALIBRATING;
        setLEDColor(COLOR_YELLOW);
        Serial.println(F("{\"type\":\"state\",\"new\":\"calibrating\"}"));
        if (!flags.mpuCalibrated) calibrateMPUOffsets();
        if (!flags.mqCalibrated) calibrateMQBaselines();
      }
      break;
      
    case STATE_CALIBRATING:
      setLEDColor(COLOR_YELLOW);
      flags.calibrationComplete = flags.mpuCalibrated && flags.mqCalibrated;
      if (flags.calibrationComplete) {
        currentState = STATE_READY;
        setLEDColor(COLOR_GREEN);
        Serial.println(F("{\"type\":\"system\",\"state\":\"ready\",\"msg\":\"All systems operational\"}"));
      }
      break;
      
    case STATE_READY: 
      setLEDColor(COLOR_GREEN); 
      break;
      
    case STATE_ERROR: 
      setLEDColor(COLOR_RED); 
      break;
  }
}

// commands stuff, i think this will do this, ayy dont mess this upp
void sendStatusReport() {
  Serial.printf("{\"type\":\"status_report\",\"t\":%lu,\"state\":\"%s\",\"pi_ready\":%s,\"override\":%s,"
                "\"mpu_cal\":%s,\"mq_cal\":%s,\"i2c_ok\":%s,\"mq2_base\":%d,\"mq5_base\":%d,"
                "\"ambient_temp\":%.2f,\"yaw\":%.2f,\"pos_x\":%.3f,\"pos_y\":%.3f,"
                "\"rpm_l\":%.2f,\"rpm_r\":%.2f,\"scan_enabled\":%s,\"pump_active\":%s}\n",
                millis(), getStateName(), 
                flags.piReady ? "true" : "false", 
                flags.manualOverride ? "true" : "false",
                flags.mpuCalibrated ? "true" : "false", 
                flags.mqCalibrated ? "true" : "false", 
                flags.i2cDevicesOK ? "true" : "false",
                mq2Baseline, mq5Baseline, ambientTemp, yawDeg, posX, posY, leftRPM, rightRPM,
                flags.scanningEnabled ? "true" : "false", 
                flags.pumpActive ? "true" : "false");
}

void processCommand(const char* cmd) {
  if (!cmd || cmd[0] == '\0') return;
  
  Serial.printf("{\"type\":\"cmd_rx\",\"cmd\":\"%s\"}\n", cmd);

  if (strcasecmp(cmd, "READY") == 0) { 
    flags.piReady = 1; 
    Serial.println(F("{\"type\":\"handshake\",\"status\":\"pi_ready\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "OVERRIDE") == 0) { 
    flags.manualOverride = 1; 
    Serial.println(F("{\"type\":\"handshake\",\"status\":\"manual_override\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "STATUS") == 0) { 
    sendStatusReport(); 
    return; 
  }
  if (strcasecmp(cmd, "REBOOT") == 0) { 
    Serial.println(F("{\"type\":\"system\",\"msg\":\"rebooting\"}")); 
    delay(100); 
    ESP.restart(); 
    return; 
  }
  if (strcasecmp(cmd, "SELFTEST") == 0) {
    Serial.println(F("{\"type\":\"self_test\",\"status\":\"starting\"}"));
    setLEDColor(COLOR_RED); delay(100); 
    setLEDColor(COLOR_GREEN); delay(100); 
    setLEDColor(COLOR_BLUE); delay(100);
    
    bool tmpOK = checkI2CDevice(TMP117_ADDR);
    bool mpuOK = checkI2CDevice(MPU6050_ADDR);
    Serial.printf("{\"type\":\"self_test\",\"component\":\"i2c\",\"device\":\"TMP117\",\"status\":\"%s\"}\n", tmpOK ? "ok" : "fail");
    Serial.printf("{\"type\":\"self_test\",\"component\":\"i2c\",\"device\":\"MPU6050\",\"status\":\"%s\"}\n", mpuOK ? "ok" : "fail");
    
    int m2r = analogRead(MQ2_PIN), m5r = analogRead(MQ5_PIN);
    Serial.printf("{\"type\":\"self_test\",\"component\":\"mq_sensors\",\"mq2_raw\":%d,\"mq5_raw\":%d,\"status\":\"%s\"}\n",
                  m2r, m5r, (m2r > 10 && m2r < 4085 && m5r > 10 && m5r < 4085) ? "ok" : "warn");
    return;
  }

  if (currentState != STATE_READY && !flags.manualOverride) {
    Serial.printf("{\"type\":\"error\",\"msg\":\"not_ready\",\"state\":\"%s\"}\n", getStateName()); 
    return;
  }

  if (strcasecmp(cmd, "STOP") == 0) { 
    motorStop(); 
    Serial.println(F("{\"ack\":\"stop\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "FORWARD") == 0) { 
    setLeftMotor(60); setRightMotor(60); applyMotorSpeeds(); 
    Serial.println(F("{\"ack\":\"forward\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "BACKWARD") == 0) { 
    setLeftMotor(-60); setRightMotor(-60); applyMotorSpeeds(); 
    Serial.println(F("{\"ack\":\"backward\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "LEFT") == 0) { 
    setLeftMotor(-80); setRightMotor(80); applyMotorSpeeds(); 
    delay(260); motorStop(); 
    Serial.println(F("{\"ack\":\"left\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "RIGHT") == 0) { 
    setLeftMotor(80); setRightMotor(-80); applyMotorSpeeds(); 
    delay(260); motorStop(); 
    Serial.println(F("{\"ack\":\"right\"}")); 
    return; 
  }

  if (strncasecmp(cmd, "L ", 2) == 0) {
    int L = atoi(cmd + 2); 
    setLeftMotor(L); applyMotorSpeeds();
    Serial.printf("{\"ack\":\"motor_l\",\"speed\":%d}\n", L); 
    return;
  }
  if (strncasecmp(cmd, "R ", 2) == 0) {
    int R = atoi(cmd + 2); 
    setRightMotor(R); applyMotorSpeeds();
    Serial.printf("{\"ack\":\"motor_r\",\"speed\":%d}\n", R); 
    return;
  }
  if (strncasecmp(cmd, "DRIVE ", 6) == 0) {
    const char* space = strchr(cmd + 6, ' ');
    if (space) { 
      int L = atoi(cmd + 6); 
      int R = atoi(space + 1);
      setLeftMotor(L); setRightMotor(R); applyMotorSpeeds();
      Serial.printf("{\"ack\":\"drive\",\"l\":%d,\"r\":%d}\n", L, R); 
    }
    return;
  }

  if (strcasecmp(cmd, "PUMP_ON") == 0) { 
    flags.pumpActive = 1; 
    digitalWrite(PUMP_PIN, HIGH); 
    pumpStartTime = millis(); 
    Serial.println(F("{\"ack\":\"pump_on\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "PUMP_OFF") == 0) { 
    pumpOff(); 
    Serial.println(F("{\"ack\":\"pump_off\"}")); 
    return; 
  }

  if (strncasecmp(cmd, "TILT_CAMERA ", 12) == 0) { 
    int a = atoi(cmd + 12); 
    tiltAngle.target = clampAngle(a); 
    Serial.printf("{\"ack\":\"tilt\",\"angle\":%d}\n", tiltAngle.target); 
    return; 
  }
  if (strncasecmp(cmd, "TURRET ", 7) == 0) { 
    int a = atoi(cmd + 7);  
    turretAngle.target = clampAngle(a); 
    Serial.printf("{\"ack\":\"turret\",\"angle\":%d}\n", turretAngle.target); 
    return; 
  }
  if (strncasecmp(cmd, "SCAN_SERVO ", 11) == 0) { 
    int a = atoi(cmd + 11); 
    scanAngle.target = clampAngle(a); 
    Serial.printf("{\"ack\":\"scan_servo\",\"angle\":%d}\n", scanAngle.target); 
    return; 
  }

  if (strcasecmp(cmd, "SCAN_FIRE") == 0) { 
    scanMode = SCAN_FIRE; 
    flags.scanningEnabled = 1; 
    scanState = MOVING_TO_LEFT; 
    Serial.println(F("{\"ack\":\"scan_fire_start\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "SCAN") == 0) { 
    scanMode = SCAN_DIRECTION; 
    flags.scanningEnabled = 1; 
    leftScanned = rightScanned = false; 
    scanState = MOVING_TO_LEFT; 
    scanAngle.target = SCAN_MIN; 
    Serial.println(F("{\"ack\":\"scan_direction_start\"}")); 
    return; 
  }
  if (strcasecmp(cmd, "DISABLE") == 0) { 
    flags.scanningEnabled = 0; 
    flags.samplingEndpoint = 0; 
    pumpOff(); 
    Serial.println(F("{\"ack\":\"disable\"}")); 
    return; 
  }

  if (strcasecmp(cmd, "CAL_MQ") == 0) { 
    calibrateMQBaselines(); 
    return; 
  }
  if (strcasecmp(cmd, "RESET_ODOM") == 0) { 
    posX = posY = yawDeg = 0; 
    leftPulses = rightPulses = 0; 
    lastLeftPulseCount = lastRightPulseCount = 0; 
    Serial.println(F("{\"ack\":\"odom_reset\"}")); 
    return; 
  }

  Serial.printf("{\"type\":\"error\",\"msg\":\"unknown_cmd\",\"cmd\":\"%s\"}\n", cmd);
}

// watchdog initialization stuff, i think this will do this, ayy dont mess this upp
void initWatchdog() {
  Serial.println(F("{\"type\":\"watchdog\",\"status\":\"initializing\"}"));
  
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  
  esp_err_t err = esp_task_wdt_init(&wdt_config);
  if (err == ESP_OK) {
    err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) {
      Serial.printf("{\"type\":\"watchdog\",\"status\":\"active\",\"timeout_sec\":%d}\n", WDT_TIMEOUT_SEC);
    } else {
      Serial.println(F("{\"type\":\"watchdog\",\"status\":\"add_failed\"}"));
    }
  } else if (err == ESP_ERR_INVALID_STATE) {
    err = esp_task_wdt_add(NULL);
    Serial.printf("{\"type\":\"watchdog\",\"status\":\"reused\",\"timeout_sec\":%d}\n", WDT_TIMEOUT_SEC);
  } else {
    Serial.println(F("{\"type\":\"watchdog\",\"status\":\"init_failed\"}"));
  }
}

// setup stuff, i think this will do this, ayy dont mess this upp
void setup() {
  Serial.begin(115200);
  delay(300);
  bootStartTime = millis();
  Serial.println(F("{\"type\":\"boot\",\"state\":\"init\",\"msg\":\"ESP32-S3 Firebot v2.3 starting\"}"));

  initWatchdog();

  statusLED.begin(); 
  statusLED.setBrightness(255); 
  setLEDColor(COLOR_BLUE);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
  Wire.setClock(100000);

  pinMode(MQ2_PIN, INPUT); 
  pinMode(MQ5_PIN, INPUT); 
  pinMode(FLAME_D0, INPUT);
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP); 
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), IR_left_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), IR_right_ISR, FALLING);

  pinMode(PUMP_PIN, OUTPUT); 
  digitalWrite(PUMP_PIN, LOW);

  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT);
  pinMode(ENA, OUTPUT);  
  pinMode(ENB, OUTPUT);
  motorsOff();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  scanServo.setPeriodHertz(50);   
  scanServo.attach(SERVO_SCAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  turretServo.setPeriodHertz(50); 
  turretServo.attach(SERVO_TURRET_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.setPeriodHertz(50);   
  tiltServo.attach(SERVO_TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  centerAllServos();

  esp_task_wdt_reset();

  currentState = STATE_SELF_TEST;
  Serial.println(F("{\"type\":\"state\",\"new\":\"self_test\"}"));
  setLEDColor(COLOR_RED); delay(100); 
  setLEDColor(COLOR_GREEN); delay(100); 
  setLEDColor(COLOR_BLUE); delay(100);

  bool tmpOK = checkI2CDevice(TMP117_ADDR);
  bool mpuOK = checkI2CDevice(MPU6050_ADDR);
  Serial.printf("{\"type\":\"self_test\",\"component\":\"i2c\",\"device\":\"TMP117\",\"status\":\"%s\"}\n", tmpOK ? "ok" : "fail");
  Serial.printf("{\"type\":\"self_test\",\"component\":\"i2c\",\"device\":\"MPU6050\",\"status\":\"%s\"}\n", mpuOK ? "ok" : "fail");
  flags.i2cDevicesOK = tmpOK && mpuOK;

  if (!flags.i2cDevicesOK) {
    currentState = STATE_ERROR; 
    setLEDColor(COLOR_RED);
    Serial.println(F("{\"type\":\"error\",\"code\":\"I2C_FAIL\",\"msg\":\"I2C device(s) missing\"}"));
  } else {
    mpu.initialize();
    currentState = STATE_WAITING_PI; 
    setLEDColor(COLOR_BLUE);
    Serial.println(F("{\"type\":\"state\",\"new\":\"waiting_pi\"}"));
  }
  
  esp_task_wdt_reset();
  Serial.println(F("{\"type\":\"boot\",\"msg\":\"Setup complete, waiting for Pi\"}"));
}

// loop stuff, i think this will do this, ayy dont mess this upp
void loop() {
  uint32_t now = millis();

  esp_task_wdt_reset();

  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 32 && c <= 126) {
      if (incomingIdx < sizeof(incomingBuffer) - 1) {
        incomingBuffer[incomingIdx++] = c;
        incomingBuffer[incomingIdx] = '\0';
      }
      lastCharTime = now;
    }
    if (c == '\n' || c == '\r') {
      if (incomingIdx > 0) {
        while (incomingIdx > 0 && incomingBuffer[incomingIdx - 1] == ' ') {
          incomingBuffer[--incomingIdx] = '\0';
        }
        processCommand(incomingBuffer);
        incomingIdx = 0;
        incomingBuffer[0] = '\0';
      }
    }
  }
  
  if (incomingIdx > 0 && (now - lastCharTime > 800)) { 
    processCommand(incomingBuffer); 
    incomingIdx = 0;
    incomingBuffer[0] = '\0';
  }

  updateStateMachine();

  if (flags.i2cDevicesOK) {
    updateMPU();
  }

  if (currentState == STATE_READY || flags.manualOverride) {
    computeRPM();
    updateOdometry();
    updatePump();
    updateServos();
    handleScanningTick(now);
  } else {
    updateServos();
  }

  updateSoftwarePWM();

  delayMicroseconds(500);
}