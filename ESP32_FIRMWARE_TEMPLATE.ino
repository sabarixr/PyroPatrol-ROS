/*
 * Fire-Fighting Robot - ESP32 Firmware
 * 
 * This firmware should be uploaded to your ESP32 to work with the ROS 2 system.
 * 
 * Hardware:
 * - L298N Motor Driver: GPIO 27, 22, 23, 24, 17, 18
 * - Wheel Encoders: GPIO 13 (left), GPIO 14 (right)
 * - MPU6050 IMU: I2C (SDA=21, SCL=22)
 * - MQ2 Smoke Sensor: Analog pin
 * - MQ5 Gas Sensor: Analog pin
 * - Flame Sensor: Analog pin
 * - Temperature Sensor: Analog pin
 * - Water Pump Relay: GPIO pin
 * 
 * Serial Communication:
 * - Baud: 115200
 * - Protocol: Text commands + JSON telemetry
 * 
 * Commands Accepted:
 * - DRIVE <left> <right>   # -100 to 100 for each wheel
 * - STOP                   # Stop both motors
 * - SCAN                   # Read fire sensors
 * - PUMP_ON                # Activate water pump
 * - PUMP_OFF               # Deactivate water pump
 * - STATUS                 # Get full telemetry
 * - DISABLE                # Disable motors
 */

#include <Wire.h>

// ===== MPU6050 I2C ADDRESS =====
#define MPU6050_ADDR 0x68

// ===== MOTOR PINS (L298N) =====
#define MOTOR_LEFT_PWM 27
#define MOTOR_LEFT_IN1 22
#define MOTOR_LEFT_IN2 23
#define MOTOR_RIGHT_PWM 24
#define MOTOR_RIGHT_IN1 17
#define MOTOR_RIGHT_IN2 18

// ===== ENCODER PINS =====
#define ENCODER_LEFT 13
#define ENCODER_RIGHT 14

// ===== SENSOR PINS =====
#define MQ2_PIN 34       // Smoke sensor (analog)
#define MQ5_PIN 35       // Gas sensor (analog)
#define FLAME_PIN 32     // Flame sensor (analog)
#define TEMP_PIN 33      // Temperature sensor (analog)
#define PUMP_PIN 25      // Water pump relay

// ===== ENCODER VARIABLES =====
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long lastEncoderReport = 0;
const unsigned long ENCODER_REPORT_INTERVAL = 100; // 10 Hz

// ===== MPU6050 VARIABLES =====
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temperature;

// ===== TIMING =====
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 100; // 10 Hz

// ===== INTERRUPT HANDLERS =====
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

// ===== MPU6050 FUNCTIONS =====
void initMPU6050() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 initialized");
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true); // Request 14 bytes
  
  // Read accelerometer (3 axes, 2 bytes each)
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  
  // Read temperature (2 bytes)
  temperature = Wire.read() << 8 | Wire.read();
  
  // Read gyroscope (3 axes, 2 bytes each)
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// ===== MOTOR CONTROL =====
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed) * 255 / 100);
  } else if (leftSpeed < 0) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed) * 255 / 100);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_PWM, 0);
  }
  
  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed) * 255 / 100);
  } else if (rightSpeed < 0) {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed) * 255 / 100);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, 0);
  }
}

void stopMotors() {
  setMotorSpeed(0, 0);
}

// ===== SENSOR READING =====
void readSensors() {
  int mq2 = analogRead(MQ2_PIN);
  int mq5 = analogRead(MQ5_PIN);
  int flame = analogRead(FLAME_PIN);
  int temp = analogRead(TEMP_PIN);
  
  Serial.print("SENSORS: MQ2=");
  Serial.print(mq2);
  Serial.print(" MQ5=");
  Serial.print(mq5);
  Serial.print(" FLAME=");
  Serial.print(flame);
  Serial.print(" TEMP=");
  Serial.println(temp);
}

// ===== TELEMETRY =====
void sendTelemetry() {
  // Read MPU6050
  readMPU6050();
  
  // Read fire sensors
  int mq2 = analogRead(MQ2_PIN);
  int mq5 = analogRead(MQ5_PIN);
  int flame = analogRead(FLAME_PIN);
  int temp = analogRead(TEMP_PIN);
  
  // Calculate acceleration in g's (16384 LSB/g for ±2g range)
  float ax = accelX / 16384.0;
  float ay = accelY / 16384.0;
  float az = accelZ / 16384.0;
  
  // Calculate angular velocity in deg/s (131 LSB/(deg/s) for ±250deg/s range)
  float gx = gyroX / 131.0;
  float gy = gyroY / 131.0;
  float gz = gyroZ / 131.0;
  
  // Calculate temperature in Celsius
  float tempC = temperature / 340.0 + 36.53;
  
  // Send JSON telemetry
  Serial.print("{");
  Serial.print("\"mq2\":");
  Serial.print(mq2);
  Serial.print(",\"mq5\":");
  Serial.print(mq5);
  Serial.print(",\"flame\":");
  Serial.print(flame);
  Serial.print(",\"temp\":");
  Serial.print(temp);
  Serial.print(",\"encoders\":{\"left\":");
  Serial.print(leftEncoderCount);
  Serial.print(",\"right\":");
  Serial.print(rightEncoderCount);
  Serial.print("},\"imu\":{\"ax\":");
  Serial.print(ax, 3);
  Serial.print(",\"ay\":");
  Serial.print(ay, 3);
  Serial.print(",\"az\":");
  Serial.print(az, 3);
  Serial.print(",\"gx\":");
  Serial.print(gx, 2);
  Serial.print(",\"gy\":");
  Serial.print(gy, 2);
  Serial.print(",\"gz\":");
  Serial.print(gz, 2);
  Serial.print(",\"temp_c\":");
  Serial.print(tempC, 1);
  Serial.print("}}");
  Serial.println();
}

// ===== COMMAND PROCESSING =====
void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("DRIVE ")) {
    // Parse: DRIVE <left> <right>
    int space = cmd.indexOf(' ', 6);
    if (space > 0) {
      int leftSpeed = cmd.substring(6, space).toInt();
      int rightSpeed = cmd.substring(space + 1).toInt();
      
      // Constrain speeds
      leftSpeed = constrain(leftSpeed, -100, 100);
      rightSpeed = constrain(rightSpeed, -100, 100);
      
      setMotorSpeed(leftSpeed, rightSpeed);
      Serial.print("OK: DRIVE ");
      Serial.print(leftSpeed);
      Serial.print(" ");
      Serial.println(rightSpeed);
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
    Serial.println("OK: STOP");
  }
  else if (cmd == "SCAN") {
    readSensors();
  }
  else if (cmd == "PUMP_ON") {
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("OK: PUMP_ON");
  }
  else if (cmd == "PUMP_OFF") {
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("OK: PUMP_OFF");
  }
  else if (cmd == "STATUS") {
    sendTelemetry();
  }
  else if (cmd == "DISABLE") {
    stopMotors();
    Serial.println("OK: DISABLE");
  }
  else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(cmd);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  // Encoder pins
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);
  
  // Sensor pins
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ5_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  
  // Pump pin
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  
  // Initialize MPU6050
  initMPU6050();
  
  Serial.println("ESP32 Fire-Fighting Robot Ready");
  Serial.println("Commands: DRIVE <L> <R>, STOP, SCAN, PUMP_ON, PUMP_OFF, STATUS, DISABLE");
}

// ===== MAIN LOOP =====
void loop() {
  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Send periodic telemetry
  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    sendTelemetry();
    lastTelemetry = now;
  }
}
