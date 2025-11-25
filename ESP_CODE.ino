#include <Wire.h>

// ---------------- PIN MAP ----------------
#define TMP_SDA 21
#define TMP_SCL 22
#define TMP117_ADDR 0x48

#define MQ2_PIN   32
#define MQ5_PIN   33

#define FLAME_D0  19

#define SERVO_SCAN_PIN   25
#define SERVO_TURRET_PIN 26

// LEDC Channels for servos
#define SCAN_LEDC_CHANNEL  0
#define TURRET_LEDC_CHANNEL 1
#define LEDC_FREQ 50  // 50Hz for servos
#define LEDC_RESOLUTION 16  // 16-bit resolution

#define PUMP_PIN 27

#define AIN1 16
#define AIN2 17
#define PWMA 4

#define BIN1 18
#define BIN2 23
#define PWMB 5

// ---------------- SCAN CONFIG ----------------
const int SCAN_MIN = 20;
const int SCAN_MAX = 160;
const int SWEEP_STEP = 5;
const unsigned long SWEEP_INTERVAL_MS = 50;
const unsigned long SAMPLE_INTERVAL_MS = 300;  // Time between samples

// ---------------- SENSOR THRESHOLDS ----------------
#define MQ2_THRESHOLD 1500  // Raw ADC value threshold
#define MQ5_THRESHOLD 500   // Raw ADC value threshold
#define TEMP_RISE_MIN 2.0

const int MIN_SAMPLES = 15;
const int NOISE_FLOOR = 100;  // Minimum raw value to consider

// ---------------- MOTOR MOVEMENT CONFIG ----------------
const int TURN_FORWARD_SPEED = 70;
const int TURN_BACKWARD_SPEED = -70;
const int TURN_FORWARD_MS = 200;
const int TURN_BACKWARD_MS = 150;
const int TURN_ROTATION_MS = 300;

// ---------------- STRUCTS ----------------
struct ScanResult {
  float mq2_avg = 0;
  float mq5_avg = 0;
  float temp_avg = 0;
  int flame_hits = 0;
  float score = 0;
  int angle = 0;
  String dominant = "";
};

// ---------------- GLOBALS ----------------
int currentScanAngle = 90;
int targetScanAngle = SCAN_MIN;
int currentTurretAngle = 90;
int targetTurretAngle = 90;

int scanAngle = SCAN_MIN;
bool scanningForward = true;
enum ScanState { MOVING_TO_LEFT, AT_LEFT, MOVING_TO_CENTER1, AT_CENTER1, MOVING_TO_RIGHT, AT_RIGHT, MOVING_TO_CENTER2, AT_CENTER2, SCAN_COMPLETE };
ScanState scanState = MOVING_TO_LEFT;
enum ScanMode { SCAN_FIRE, SCAN_DIRECTION };
ScanMode scanMode = SCAN_FIRE;

bool samplingEndpoint = false;
unsigned long endpointReachedAt = 0;
int endpointAngle = 0;

int samplesTaken = 0;
float mq2Acc = 0, mq5Acc = 0, tempAcc = 0;
int flameCount = 0;

float ambientTemp = 25.0;

String incoming = "";
unsigned long lastCharTime = 0;

bool scanningEnabled = false;
bool pumpActive = false;
unsigned long pumpStartTime = 0;
const unsigned long PUMP_DURATION = 3000;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int mq2Baseline = 0;
int mq5Baseline = 0;

ScanResult leftScan;
ScanResult rightScan;
bool leftScanned = false;
bool rightScanned = false;

// ---------------- MOTOR CONTROL -------------------
int mapSpeedMagnitude(int s) {
  s = constrain(s, -100, 100);
  int mag = abs(s);
  return (mag * 255) / 100;
}

void applyMotorSpeeds() {
  if (leftMotorSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  if (rightMotorSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMA, mapSpeedMagnitude(leftMotorSpeed));
  analogWrite(PWMB, mapSpeedMagnitude(rightMotorSpeed));
}

void motorStop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
}

// ---------------- JERKY TURN FUNCTIONS ----------------
void executeLeftTurn() {
  Serial.println("{\"type\":\"status\",\"msg\":\"Executing left turn with jerk motion\"}");
  
  // Forward jerk
  leftMotorSpeed = TURN_FORWARD_SPEED;
  rightMotorSpeed = TURN_FORWARD_SPEED;
  applyMotorSpeeds();
  delay(TURN_FORWARD_MS);
  
  // Backward jerk
  leftMotorSpeed = TURN_BACKWARD_SPEED;
  rightMotorSpeed = TURN_BACKWARD_SPEED;
  applyMotorSpeeds();
  delay(TURN_BACKWARD_MS);
  
  // Rotate left
  leftMotorSpeed = 10;
  rightMotorSpeed = 100;
  applyMotorSpeeds();
  delay(TURN_ROTATION_MS);
  
  motorStop();
}

void executeRightTurn() {
  Serial.println("{\"type\":\"status\",\"msg\":\"Executing right turn with jerk motion\"}");
  
  // Forward jerk
  leftMotorSpeed = TURN_FORWARD_SPEED;
  rightMotorSpeed = TURN_FORWARD_SPEED;
  applyMotorSpeeds();
  delay(TURN_FORWARD_MS);
  
  // Backward jerk
  leftMotorSpeed = TURN_BACKWARD_SPEED;
  rightMotorSpeed = TURN_BACKWARD_SPEED;
  applyMotorSpeeds();
  delay(TURN_BACKWARD_MS);
  
  // Rotate right
  leftMotorSpeed = 100;
  rightMotorSpeed = 10;
  applyMotorSpeeds();
  delay(TURN_ROTATION_MS);
  
  motorStop();
}

// ---------------- SERVO LEDC CONTROL ----------------
void servoWriteMicroseconds(int pin, int angle) {
  // Convert angle (0-180) to pulse width (500-2500 microseconds)
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  // Convert to duty cycle for 16-bit resolution at 50Hz
  int dutyCycle = (pulseWidth * 65536) / 20000;
  ledcWrite(pin, dutyCycle);
}

void smoothServoMove(int pin, int &current, int target, int speed = 2) {
  if (current < target) {
    current += min(speed, target - current);
  } else if (current > target) {
    current -= min(speed, current - target);
  }
  servoWriteMicroseconds(pin, current);
}

void setServoAngle(int pin, int &current, int target) {
  current = constrain(target, 20, 160);
  servoWriteMicroseconds(pin, current);
}

void updateServos() {
  // Smooth transitions for both servos
  // Very slow scan servo movement for 700g load - 0.5 degrees per cycle
  if (scanningEnabled) {
    smoothServoMove(SERVO_SCAN_PIN, currentScanAngle, targetScanAngle, 1);  // Slow for heavy load
  } else {
    smoothServoMove(SERVO_SCAN_PIN, currentScanAngle, targetScanAngle, 2);
  }
  smoothServoMove(SERVO_TURRET_PIN, currentTurretAngle, targetTurretAngle, 5);
}

// ---------------- SENSOR HELPERS ------------------
float readTMP117() {
  static float lastValidTemp = 25.0;
  static unsigned long lastReadTime = 0;
  
  if (millis() - lastReadTime < 300) return lastValidTemp;
  
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(TMP117_ADDR);
    Wire.write(0x00);
    byte error = Wire.endTransmission(false);
    if (error == 0) {
      int received = Wire.requestFrom(TMP117_ADDR, 2);
      if (received == 2) {
        int16_t raw = (Wire.read() << 8) | Wire.read();
        float temp = raw * 0.0078125f;
        if (temp > -40.0 && temp < 125.0) {
          lastValidTemp = temp;
          lastReadTime = millis();
          return temp;
        }
      }
    }
    delay(10);
  }
  return lastValidTemp;
}

float normalizeADC(int raw) {
  return constrain(raw / 4095.0f, 0.0f, 1.0f);
}

float tempDeltaScore(float t) {
  return constrain((t - ambientTemp) / 20.0f, 0.0f, 1.0f);
}

// ---------------- SCORING -------------------
float computeScore(ScanResult &s) {
  float smoke = normalizeADC((int)constrain(round(s.mq2_avg), 0, 4095));
  float gas   = normalizeADC((int)constrain(round(s.mq5_avg), 0, 4095));
  float tempS = tempDeltaScore(s.temp_avg);

  if (s.flame_hits >= 2) {
    s.dominant = "flame";
    return s.score = 1.0 + 0.4 * smoke + 0.2 * tempS + 0.05 * gas;
  }
  if (smoke > 0.4) {
    s.dominant = "smoke";
    return s.score = 0.7 * smoke + 0.3 * tempS;
  }
  if (tempS > 0.2) {
    s.dominant = "heat";
    return s.score = 0.5 * tempS + 0.3 * smoke + 0.2 * gas;
  }
  s.dominant = "none";
  return s.score = 0.3 * smoke + 0.3 * gas + 0.4 * tempS;
}

// ---------------- SAMPLING ------------------
void startSampling(int angle, bool shouldSample) {
  // In SCAN_FIRE mode, don't pause at all - just keep scanning
  if (scanMode == SCAN_FIRE) {
    samplingEndpoint = false;
    return;
  }
  
  // In SCAN_DIRECTION mode, pause and collect samples
  samplingEndpoint = true;
  endpointAngle = angle;
  samplesTaken = 0;
  mq2Acc = mq5Acc = tempAcc = 0;
  flameCount = 0;
  targetScanAngle = angle;
  
  // Debug output
  Serial.printf("{\"type\":\"debug\",\"msg\":\"Starting sampling\",\"angle\":%d,\"shouldSample\":%d}\n", 
                angle, shouldSample);
  
  // Only collect samples at left/right endpoints
  if (!shouldSample) {
    samplesTaken = -1;  // Flag to indicate "no sampling needed, just pause briefly"
  }
}

void finishSampling() {
  // If this was a non-sampling pause (center position), just exit
  if (samplesTaken <= 0) {
    samplingEndpoint = false;
    Serial.printf("{\"type\":\"debug\",\"msg\":\"Pause complete (no sampling)\",\"angle\":%d}\n", endpointAngle);
    return;
  }
  
  ScanResult result;
  result.mq2_avg = mq2Acc / samplesTaken;
  result.mq5_avg = mq5Acc / samplesTaken;
  result.temp_avg = tempAcc / samplesTaken;
  result.flame_hits = flameCount;
  result.angle = endpointAngle;
  computeScore(result);
  
  // Store left or right
  if (endpointAngle == SCAN_MIN) {
    leftScan = result;
    leftScanned = true;
    Serial.printf("{\"type\":\"debug\",\"msg\":\"Left scan stored\",\"samples\":%d}\n", samplesTaken);
  } else if (endpointAngle == SCAN_MAX) {
    rightScan = result;
    rightScanned = true;
    Serial.printf("{\"type\":\"debug\",\"msg\":\"Right scan stored\",\"samples\":%d}\n", samplesTaken);
  }
  
  sendScanResultJSON(result);
  samplingEndpoint = false;
}

void transitionToNextState() {
  // Transition to next state based on current state
  if (scanState == AT_LEFT) {
    scanState = MOVING_TO_CENTER1;
    Serial.println("{\"type\":\"status\",\"msg\":\"Moving to center...\"}");
  } else if (scanState == AT_CENTER1) {
    scanState = MOVING_TO_RIGHT;
    Serial.println("{\"type\":\"status\",\"msg\":\"Moving to right...\"}");
  } else if (scanState == AT_RIGHT) {
    scanState = MOVING_TO_CENTER2;
    Serial.println("{\"type\":\"status\",\"msg\":\"Returning to center...\"}");
  } else if (scanState == AT_CENTER2) {
    scanState = SCAN_COMPLETE;
    sendScanDirectionJSON();
    scanningEnabled = false;
    Serial.println("{\"type\":\"status\",\"msg\":\"Scan complete\"}");
  }
}

// ---------------- PUMP CONTROL ------------------
void startPump(int angle) {
  if (pumpActive) return;
  targetTurretAngle = angle;
  digitalWrite(PUMP_PIN, HIGH);
  pumpActive = true;
  pumpStartTime = millis();
  Serial.printf("{\"type\":\"pump\",\"status\":\"on\",\"angle\":%d}\n", angle);
}

void updatePump() {
  if (pumpActive && (millis() - pumpStartTime >= PUMP_DURATION)) {
    digitalWrite(PUMP_PIN, LOW);
    targetTurretAngle = 90;
    pumpActive = false;
    Serial.println("{\"type\":\"pump\",\"status\":\"off\"}");
  }
}

// ---------------- JSON OUTPUT ------------------
void sendScanSampleJSON(int angle, int m2, int m5, float t, int flameD, int sampleNo) {
  Serial.printf(
    "{\"type\":\"scan_sample\",\"t\":%lu,\"angle\":%d,\"mq2\":%d,\"mq5\":%d,\"temp\":%.2f,\"flame\":%d,\"sample\":%d}\n",
    millis(), angle, m2, m5, t, flameD, sampleNo
  );
}

void sendScanResultJSON(ScanResult &r) {
  Serial.printf(
    "{\"type\":\"scan_result\",\"t\":%lu,\"angle\":%d,\"mq2_avg\":%.1f,\"mq5_avg\":%.1f,"
    "\"temp_avg\":%.2f,\"flame_hits\":%d,\"score\":%.3f,\"dominant\":\"%s\"}\n",
    millis(), r.angle, r.mq2_avg, r.mq5_avg, r.temp_avg, r.flame_hits, r.score, r.dominant.c_str()
  );
}

void sendScanDirectionJSON() {
  // Compare left and right scans
  bool leftSignificant = leftScanned && (leftScan.score > 0.3 || leftScan.flame_hits > 0);
  bool rightSignificant = rightScanned && (rightScan.score > 0.3 || rightScan.flame_hits > 0);
  
  String direction = "none";
  String reason = "No significant detection";
  ScanResult *best = nullptr;
  
  if (leftSignificant || rightSignificant) {
    if (leftScan.score > rightScan.score) {
      direction = "left";
      best = &leftScan;
      reason = "Higher score on left side";
    } else if (rightScan.score > leftScan.score) {
      direction = "right";
      best = &rightScan;
      reason = "Higher score on right side";
    } else {
      // Equal scores - check averages
      float leftAvg = (leftScan.mq2_avg + leftScan.mq5_avg) / 2.0;
      float rightAvg = (rightScan.mq2_avg + rightScan.mq5_avg) / 2.0;
      if (leftAvg > rightAvg) {
        direction = "left";
        best = &leftScan;
        reason = "Higher sensor average on left";
      } else {
        direction = "right";
        best = &rightScan;
        reason = "Higher sensor average on right";
      }
    }
  } else {
    // No significant detection - suggest area with higher average
    float leftAvg = (leftScan.mq2_avg + leftScan.mq5_avg) / 2.0;
    float rightAvg = (rightScan.mq2_avg + rightScan.mq5_avg) / 2.0;
    if (leftAvg > rightAvg && leftAvg > 200) {
      direction = "left";
      best = &leftScan;
      reason = "Slightly higher readings on left";
    } else if (rightAvg > leftAvg && rightAvg > 200) {
      direction = "right";
      best = &rightScan;
      reason = "Slightly higher readings on right";
    }
  }
  
  Serial.printf(
    "{\"type\":\"scan_complete\",\"t\":%lu,\"direction\":\"%s\",\"reason\":\"%s\",",
    millis(), direction.c_str(), reason.c_str()
  );
  
  if (best != nullptr) {
    Serial.printf(
      "\"angle\":%d,\"score\":%.3f,\"mq2_avg\":%.1f,\"mq5_avg\":%.1f,\"temp_avg\":%.2f,\"flame_hits\":%d,\"dominant\":\"%s\",",
      best->angle, best->score, best->mq2_avg, best->mq5_avg, best->temp_avg, best->flame_hits, best->dominant.c_str()
    );
  }
  
  Serial.printf(
    "\"left\":{\"angle\":%d,\"score\":%.3f,\"mq2\":%.1f,\"mq5\":%.1f,\"temp\":%.2f},",
    leftScan.angle, leftScan.score, leftScan.mq2_avg, leftScan.mq5_avg, leftScan.temp_avg
  );
  Serial.printf(
    "\"right\":{\"angle\":%d,\"score\":%.3f,\"mq2\":%.1f,\"mq5\":%.1f,\"temp\":%.2f}}\n",
    rightScan.angle, rightScan.score, rightScan.mq2_avg, rightScan.mq5_avg, rightScan.temp_avg
  );
}

// ---------------- COMMAND PROCESSOR ----------------
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // Motor speed commands
  if (cmd.startsWith("L ")) {
    leftMotorSpeed = cmd.substring(2).toInt();
    applyMotorSpeeds();
    Serial.printf("{\"type\":\"ack\",\"cmd\":\"left_motor\",\"speed\":%d}\n", leftMotorSpeed);
    return;
  }

  if (cmd.startsWith("R ")) {
    rightMotorSpeed = cmd.substring(2).toInt();
    applyMotorSpeeds();
    Serial.printf("{\"type\":\"ack\",\"cmd\":\"right_motor\",\"speed\":%d}\n", rightMotorSpeed);
    return;
  }

  if (cmd.startsWith("SPEED ")) {
    int s = cmd.substring(6).toInt();
    leftMotorSpeed = rightMotorSpeed = s;
    applyMotorSpeeds();
    Serial.printf("{\"type\":\"ack\",\"cmd\":\"speed\",\"value\":%d}\n", s);
    return;
  }

  if (cmd.startsWith("DRIVE ")) {
    int sp1 = cmd.indexOf(' ', 6);
    if (sp1 > 6) {
      int L = cmd.substring(6, sp1).toInt();
      int R = cmd.substring(sp1 + 1).toInt();
      leftMotorSpeed = L;
      rightMotorSpeed = R;
      applyMotorSpeeds();
      Serial.printf("{\"type\":\"ack\",\"cmd\":\"drive\",\"left\":%d,\"right\":%d}\n", L, R);
      return;
    }
  }

  // Stop
  if (cmd == "STOP" || cmd == "S") {
    motorStop();
    Serial.println("{\"type\":\"ack\",\"cmd\":\"stop\"}");
    return;
  }

  // Pump control
  if (cmd == "PUMP_ON") {
    digitalWrite(PUMP_PIN, HIGH);
    pumpActive = true;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"pump_on\"}");
    return;
  }

  if (cmd == "PUMP_OFF") {
    digitalWrite(PUMP_PIN, LOW);
    pumpActive = false;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"pump_off\"}");
    return;
  }

  // Scan command - full direction scan with all sensors (stops any active SCAN_FIRE)
  if (cmd == "SCAN") {
    leftScanned = false;
    rightScanned = false;
    leftScan = ScanResult();
    rightScan = ScanResult();
    scanAngle = SCAN_MIN;
    targetScanAngle = SCAN_MIN;
    scanState = MOVING_TO_LEFT;
    samplingEndpoint = false;
    scanMode = SCAN_DIRECTION;
    scanningEnabled = true;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"scan\",\"status\":\"started\",\"mode\":\"direction\",\"note\":\"SCAN_FIRE stopped\"}");
    return;
  }

  // Scan fire command - passive flame detection only (runs continuously until DISABLE or SCAN)
  if (cmd == "SCAN_FIRE") {
    scanAngle = SCAN_MIN;
    targetScanAngle = SCAN_MIN;
    scanState = MOVING_TO_LEFT;
    samplingEndpoint = false;
    scanMode = SCAN_FIRE;
    scanningEnabled = true;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"scan_fire\",\"status\":\"started\",\"mode\":\"fire_only\",\"note\":\"Continuous until DISABLE or SCAN\"}");
    return;
  }

  if (cmd == "DISABLE") {
    scanningEnabled = false;
    samplingEndpoint = false;
    digitalWrite(PUMP_PIN, LOW);
    pumpActive = false;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"disable\"}");
    return;
  }

  // Jerky turn commands
  if (cmd == "LEFT" || cmd == "TURN_LEFT") {
    executeLeftTurn();
    Serial.println("{\"type\":\"ack\",\"cmd\":\"turn_left\"}");
    return;
  }

  if (cmd == "RIGHT" || cmd == "TURN_RIGHT") {
    executeRightTurn();
    Serial.println("{\"type\":\"ack\",\"cmd\":\"turn_right\"}");
    return;
  }

  // Basic movement
  if (cmd == "FORWARD" || cmd == "FWD") {
    leftMotorSpeed = rightMotorSpeed = 60;
    applyMotorSpeeds();
    Serial.println("{\"type\":\"ack\",\"cmd\":\"forward\"}");
    return;
  }

  if (cmd == "BACKWARD" || cmd == "BACK" || cmd == "BWD") {
    leftMotorSpeed = rightMotorSpeed = -60;
    applyMotorSpeeds();
    Serial.println("{\"type\":\"ack\",\"cmd\":\"backward\"}");
    return;
  }

  Serial.printf("{\"type\":\"error\",\"msg\":\"Unknown command: %s\"}\n", cmd.c_str());
}

// ---------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n{\"type\":\"init\",\"msg\":\"ESP32 Robot Controller Starting...\"}");

  // I2C init
  Wire.begin(TMP_SDA, TMP_SCL);
  Wire.setClock(100000);
  
  // I2C device scan
  Serial.println("{\"type\":\"init\",\"msg\":\"Scanning I2C bus...\"}");
  int deviceCount = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("{\"type\":\"i2c_device\",\"address\":\"0x%02X\"}\n", addr);
      deviceCount++;
    }
  }
  Serial.printf("{\"type\":\"init\",\"msg\":\"I2C scan complete\",\"devices_found\":%d}\n", deviceCount);
  
  if (deviceCount == 0) {
    Serial.println("{\"type\":\"warning\",\"msg\":\"No I2C devices found! Check TMP117 wiring.\"}");
  }

  // Sensor pins
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ5_PIN, INPUT);
  pinMode(FLAME_D0, INPUT);

  // Servo init with LEDC (ESP32 Arduino 3.x API)
  ledcAttach(SERVO_SCAN_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttach(SERVO_TURRET_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  
  // Initialize servo positions
  currentScanAngle = 90;
  targetScanAngle = 90;
  currentTurretAngle = 90;
  targetTurretAngle = 90;
  servoWriteMicroseconds(SERVO_SCAN_PIN, 90);
  servoWriteMicroseconds(SERVO_TURRET_PIN, 90);

  // Pump
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  motorStop();

  // Calibrate ambient temp
  delay(500);
  ambientTemp = readTMP117();
  Serial.printf("{\"type\":\"init\",\"ambient_temp\":%.2f}\n", ambientTemp);

  // MQ sensor warm-up (2 minutes)
  Serial.println("{\"type\":\"init\",\"msg\":\"Warming up MQ sensors for 2 minutes...\"}");
  for (int i = 0; i < 24; i++) {  // 24 x 5 seconds = 120 seconds
    delay(5000);
    int remaining = 120 - (i * 5);
    Serial.printf("{\"type\":\"warmup\",\"remaining_sec\":%d,\"mq2\":%d,\"mq5\":%d}\n", 
                  remaining, analogRead(MQ2_PIN), analogRead(MQ5_PIN));
  }

  // MQ baseline calibration (after warm-up)
  Serial.println("{\"type\":\"init\",\"msg\":\"Calibrating MQ baselines...\"}");
  long mq2sum = 0, mq5sum = 0;
  const int calSamples = 30;
  for (int i = 0; i < calSamples; i++) {
    mq2sum += analogRead(MQ2_PIN);
    mq5sum += analogRead(MQ5_PIN);
    delay(30);
  }
  mq2Baseline = mq2sum / calSamples;
  mq5Baseline = mq5sum / calSamples;
  
  Serial.printf("{\"type\":\"init\",\"mq2_baseline\":%d,\"mq5_baseline\":%d}\n", mq2Baseline, mq5Baseline);
  Serial.println("{\"type\":\"init\",\"msg\":\"Ready for commands\"}");
  Serial.print("> ");
}

// ---------------- LOOP ---------------------
void loop() {
  unsigned long now = millis();

  // Command input from Raspberry Pi
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 32 && c <= 126) {
      incoming += c;
      lastCharTime = now;
    }
    if (c == '\n' || c == '\r') {
      if (incoming.length() > 0) {
        processCommand(incoming);
        incoming = "";
        Serial.print("> ");
      }
    }
  }

  if (incoming.length() > 0 && (now - lastCharTime > 1000)) {
    processCommand(incoming);
    incoming = "";
    Serial.print("> ");
  }

  updatePump();
  updateServos();  // Smooth servo movements

  if (!scanningEnabled) {
    delay(10);
    return;
  }

  // ---------------- FLAME DETECTION ----------------
  static unsigned long lastFlameCheck = 0;
  static int flameConfirmCount = 0;
  if (now - lastFlameCheck >= 100) {
    lastFlameCheck = now;
    bool flameDetected = (digitalRead(FLAME_D0) == HIGH);

    if (flameDetected) {
      flameConfirmCount++;
      if (flameConfirmCount >= 3 && !pumpActive) {  // Require 3 consecutive readings
        // Use currentScanAngle for accurate position
        Serial.printf("{\"type\":\"alert\",\"msg\":\"Flame confirmed at angle %d\"}\n", currentScanAngle);
        startPump(currentScanAngle);
        flameConfirmCount = 0;
      }
    } else {
      flameConfirmCount = 0;  // Reset if no flame
    }
  }

  // ---------------- SERVO SWEEP STATE MACHINE ----------------
  static unsigned long lastSweep = 0;
  
  if (!samplingEndpoint && now - lastSweep >= SWEEP_INTERVAL_MS) {
    lastSweep = now;

    switch (scanState) {
      case MOVING_TO_LEFT:
        targetScanAngle = SCAN_MIN;  // Keep moving to left
        if (abs(currentScanAngle - SCAN_MIN) <= 2) {  // Close enough to target
          if (scanMode == SCAN_FIRE) {
            // In fire mode, just reverse and keep scanning
            scanState = MOVING_TO_RIGHT;
            targetScanAngle = SCAN_MAX;
          } else {
            scanState = AT_LEFT;
            currentScanAngle = SCAN_MIN;  // Force exact position
            targetScanAngle = SCAN_MIN;   // STOP servo here
            startSampling(SCAN_MIN, true);
            Serial.println("{\"type\":\"status\",\"msg\":\"At left endpoint, sampling...\"}");
          }
        }
        break;
        
      case MOVING_TO_CENTER1:
        targetScanAngle = 90;  // Keep moving to center
        if (abs(currentScanAngle - 90) <= 2) {  // Close enough to target
          scanState = AT_CENTER1;
          currentScanAngle = 90;
          targetScanAngle = 90;  // STOP servo here
          startSampling(90, false);
          Serial.println("{\"type\":\"status\",\"msg\":\"At center, pausing...\"}");
        }
        break;
        
      case MOVING_TO_RIGHT:
        targetScanAngle = SCAN_MAX;  // Keep moving to right
        if (abs(currentScanAngle - SCAN_MAX) <= 2) {  // Close enough to target
          if (scanMode == SCAN_FIRE) {
            // In fire mode, just reverse and keep scanning
            scanState = MOVING_TO_LEFT;
            targetScanAngle = SCAN_MIN;
          } else {
            scanState = AT_RIGHT;
            currentScanAngle = SCAN_MAX;  // Force exact position
            targetScanAngle = SCAN_MAX;   // STOP servo here
            startSampling(SCAN_MAX, true);
            Serial.println("{\"type\":\"status\",\"msg\":\"At right endpoint, sampling...\"}");
          }
        }
        break;
        
      case MOVING_TO_CENTER2:
        targetScanAngle = 90;  // Keep moving to center
        if (abs(currentScanAngle - 90) <= 2) {  // Close enough to target
          scanState = AT_CENTER2;
          currentScanAngle = 90;
          targetScanAngle = 90;  // STOP servo here
          startSampling(90, false);
          Serial.println("{\"type\":\"status\",\"msg\":\"At center, pausing...\"}");
        }
        break;
        
      case AT_LEFT:
      case AT_CENTER1:
      case AT_RIGHT:
      case AT_CENTER2:
        // Waiting for sampling to complete
        break;
        
      case SCAN_COMPLETE:
        // Already sent results
        break;
    }
  }

  // ---------------- ENDPOINT SAMPLING ----------------
  if (samplingEndpoint) {
    static unsigned long lastSample = 0;

    // Take samples during pause (only if shouldSample=true, indicated by samplesTaken >= 0)
    // Sample every 300ms until we have MIN_SAMPLES (15)
    if (scanMode == SCAN_DIRECTION && samplesTaken >= 0 && now - lastSample >= SAMPLE_INTERVAL_MS) {
      lastSample = now;

      int raw_m2 = analogRead(MQ2_PIN);
      int raw_m5 = analogRead(MQ5_PIN);
      
      // Use raw values directly (no baseline subtraction)
      int m2 = raw_m2;
      int m5 = raw_m5;

      float t = readTMP117();
      int flameD = digitalRead(FLAME_D0);

      mq2Acc += m2;
      mq5Acc += m5;
      tempAcc += t;

      if (flameD == HIGH) {
        flameCount++;
        // Spray water immediately when flame detected during scan
        if (!pumpActive) {
          Serial.printf("{\"type\":\"alert\",\"msg\":\"Flame detected at angle %d during scan\"}\n", endpointAngle);
          startPump(endpointAngle);
        }
      }

      samplesTaken++;
      
      // Send with raw values for debugging
      Serial.printf(
        "{\"type\":\"scan_sample\",\"t\":%lu,\"angle\":%d,\"mq2\":%d,\"mq5\":%d,\"raw_mq2\":%d,\"raw_mq5\":%d,\"temp\":%.2f,\"flame\":%d,\"sample\":%d}\n",
        millis(), endpointAngle, m2, m5, raw_m2, raw_m5, t, flameD, samplesTaken
      );
      
      // Check if we have enough samples
      if (samplesTaken >= MIN_SAMPLES) {
        finishSampling();
        transitionToNextState();
      }
    }
    
    // For center position (samplesTaken == -1), just do a brief pause
    // Wait for servo to stabilize (1 second) then move on
    if (scanMode == SCAN_DIRECTION && samplesTaken == -1) {
      static unsigned long centerPauseStart = 0;
      if (centerPauseStart == 0) {
        centerPauseStart = now;
      }
      
      if (now - centerPauseStart >= 1000) {  // 1 second pause at center
        centerPauseStart = 0;
        finishSampling();
        transitionToNextState();
      }
    }
  }

  delay(5);
}
