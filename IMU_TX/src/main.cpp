// IMU TX - Hand-Mounted IMU Transmitter
// Reads MPU6050 and transmits orientation via custom protocol to computer

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "protocol.h"

// Hardware configuration
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 9
#define JOYSTICK_X_PIN 4       // Joystick X-axis (future use)
#define JOYSTICK_Y_PIN 5       // Joystick Y-axis (throttle)
#define JOYSTICK_BTN_PIN 3     // Joystick button (arming - AUX1)
#define LED_PIN 10             // Status LED

// Transmit configuration
#define IMU_SEND_FREQUENCY_HZ 50  // 50 Hz IMU data
#define HEARTBEAT_FREQUENCY_HZ 2  // 2 Hz heartbeat

// ESP-NOW configuration (broadcast to RX on drone)
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast to all RX

// MPU6050 IMU
MPU6050 mpu;

// State
bool imuCalibrated = false;
bool isArmed = false;
float currentThrottle = 0.0;  // From joystick Y-axis

// Timing
unsigned long lastImuSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastCalibCheckMs = 0;

const unsigned long IMU_SEND_INTERVAL = 1000 / IMU_SEND_FREQUENCY_HZ;
const unsigned long HEARTBEAT_INTERVAL = 1000 / HEARTBEAT_FREQUENCY_HZ;
const unsigned long STATUS_INTERVAL = 5000;  // Status every 5 seconds
const unsigned long CALIB_CHECK_INTERVAL = 1000;  // Check calibration every second

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n==========================================");
  Serial.println("IMU TX - Gesture Controller");
  Serial.println("==========================================\n");
  
  // Setup pins
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  
  // Initialize I2C for IMU
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  delay(100);
  
  // Initialize MPU6050
  Serial.println("[IMU] Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("[IMU] Failed to find MPU6050 sensor!");
    Serial.println("[IMU] Check wiring and I2C address (0x68)");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.println("[IMU] MPU6050 detected!");
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);      // ±250°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);      // ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);                 // 20 Hz low-pass filter
  delay(100);
  
  Serial.println("[IMU] Sensor configured and ready");
  Serial.println("[IMU] Keep hand still for 3 seconds for zero calibration...");
  
  delay(3000);  // Let sensor stabilize
  imuCalibrated = true;
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("[JOYSTICK] Analog throttle on GPIO 5");
  Serial.println("[JOYSTICK] Button (arming/AUX1) on GPIO 3");
  
  // Initialize protocol (TX mode, broadcast to RX on drone)
  if (!CustomProtocol_Init(true, broadcastMac)) {
    Serial.println("[PROTO] Protocol init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("[PROTO] ESP-NOW ready - broadcasting to drone RX");
  Serial.printf("[IMU] Transmitting at %d Hz\n", IMU_SEND_FREQUENCY_HZ);
  Serial.println("\n*** IMU TX Ready for Direct Flight ***");
  Serial.println("*** Tilt controller = Tilt drone ***");
  Serial.println("*** Joystick Y = Throttle ***");
  Serial.println("*** Button = Arm/Disarm (AUX1) ***");
  Serial.println("*** Computer NOT required for flight! ***\n");
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol
  CustomProtocol_Update();
  
  // Read joystick button (arming)
  isArmed = !digitalRead(JOYSTICK_BTN_PIN);  // Active low
  
  // Read throttle from joystick Y-axis
  int throttleRaw = analogRead(JOYSTICK_Y_PIN);
  currentThrottle = throttleRaw / 4095.0;  // ESP32 ADC is 12-bit (0-4095)
  currentThrottle = constrain(currentThrottle, 0.0, 1.0);
  
  // Apply deadzone (center position ~2048, ±100 counts)
  if (throttleRaw > 1948 && throttleRaw < 2148) {
    currentThrottle = 0.5;  // Snap to hover throttle
  }
  
  // MPU6050 doesn't have auto-calibration like BNO055
  // Just keep LED solid after initial setup
  if (!imuCalibrated && now > 5000) {
    imuCalibrated = true;
    digitalWrite(LED_PIN, HIGH);
  }
  
  // Send IMU data at configured frequency
  if (now - lastImuSendMs >= IMU_SEND_INTERVAL) {
    lastImuSendMs = now;
    
    // Read raw IMU data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Simple complementary filter for orientation
    // Convert accel to angles (roll, pitch)
    float accelX = ax / 16384.0;  // ±2g range
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    
    float roll = atan2(accelY, accelZ);
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));
    float yaw = 0.0;  // MPU6050 has no magnetometer, yaw drifts
    
    // Convert to quaternion
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    
    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;
    
    // Prepare flags
    uint8_t flags = 0;
    if (imuCalibrated) flags |= 0x01;
    if (isArmed) flags |= 0x02;  // Button = arming state (AUX1)
    
    // Send IMU data via protocol
    if (!CustomProtocol_SendImuData(qw, qx, qy, qz, currentThrottle, flags)) {
      // Send failures handled internally
    }
  }
  
  // Send heartbeat
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    CustomProtocol_SendHeartbeat();
  }
  
  // Print status
  if (now - lastStatusMs >= STATUS_INTERVAL) {
    lastStatusMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    // Read current sensor values
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    
    float roll = atan2(accelY, accelZ);
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));
    
    Serial.println("\n--- IMU TX Status ---");
    Serial.printf("MPU6050: %s\n", imuCalibrated ? "[OK]" : "[INIT]");
    Serial.printf("Roll: %.1f° | Pitch: %.1f°\n",
                  roll * 57.3, pitch * 57.3);
    
    Serial.printf("Throttle: %.2f | Armed: %s\n",
                  currentThrottle, isArmed ? "YES" : "NO");
    
    Serial.printf("Link: %s | Packets sent: %lu | Failures: %lu\n",
                  stats.linkActive ? "ACTIVE" : "DOWN",
                  stats.packetsSent, stats.sendFailures);
    Serial.println();
  }
  
  delay(1);
}

