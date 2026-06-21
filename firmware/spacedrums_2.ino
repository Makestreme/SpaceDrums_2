#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ==========================================
// CONFIGURATION
// ==========================================
//  XIAO ESP32S3 Hub MAC ADDRESS
uint8_t hubAddress[] = {0xE8, 0x06, 0x90, 0x9D, 0xA0, 0x68};

const uint8_t STICK_ID = 0; // 0 = Left Stick, 1 = Right Stick

// --- PIN CONFIGURATION ---
const int PIN_SPI_MOSI    = 35;
const int PIN_SPI_MISO    = 37;
const int PIN_SPI_SCK     = 36;
const int PIN_I2C_SDA     = 7;
const int PIN_I2C_SCL     = 8;
const int IMU1_CS         = 5;
const int IMU2_CS         = 9;
const int PIN_STATUS_LED  = 41;
const int PMIC_KILL       = 11;
const int PMIC_INT        = 12;

const uint8_t MMC5983_ADDR  = 0x30;
const uint8_t DRV2605_ADDR  = 0x5A;

// ==========================================
// FREERTOS OBJECTS
// ==========================================
SemaphoreHandle_t i2cMutex;
QueueHandle_t hitQueue;

struct HitPayload { uint8_t stick_id; uint8_t drum_id; uint8_t velocity; };

// ==========================================
// GLOBALS & STATE MACHINES
// ==========================================
Preferences prefs;
enum SystemState { 
  STATE_BOOT_WAIT, STATE_BOOT_QUICK_SNAP, STATE_NORMAL, 
  STATE_CAL_WAIT, STATE_CAL_FIG8, STATE_CAL_WAIT_SNARE, STATE_CAL_SNARE 
};
volatile SystemState sysState = STATE_BOOT_WAIT;
volatile bool buttonPressed = false;

unsigned long stateTimer = 0;
unsigned long lastLedToggle = 0;
bool ledState = false;

volatile int16_t cur_ax1, cur_ay1, cur_az1, cur_gx1, cur_gy1, cur_gz1;
volatile int16_t cur_ax2, cur_ay2, cur_az2, cur_gx2, cur_gy2, cur_gz2;

enum DrumState { STATE_IDLE, STATE_SWINGING, STATE_REFRACTORY };
DrumState stickState = STATE_IDLE;
const int16_t SWING_START_THRESHOLD = 4000;
const int16_t HIT_DECEL_THRESHOLD   = 3000;
int16_t peak_swing_velocity = 0; 
unsigned long hitTimer = 0;

float pitch = 0.0, yaw = 0.0;
float targetYawOffset = 0.0, targetPitchOffset = 0.0;
volatile float sharedMagHeading = 0.0;

float gyroBiasX1 = 0, gyroBiasY1 = 0, gyroBiasZ1 = 0;
float gyroBiasX2 = 0, gyroBiasY2 = 0, gyroBiasZ2 = 0;
float magBiasX = 0, magBiasY = 0, magBiasZ = 0;
float magScaleX = 1.0, magScaleY = 1.0, magScaleZ = 1.0;

const int MAX_MAG_SAMPLES = 300;
struct MagSample { float x, y, z; };
MagSample magBuffer[MAX_MAG_SAMPLES];
int magSampleCount = 0;
unsigned long lastMagSampleTime = 0;

long calibGyroX1 = 0, calibGyroY1 = 0, calibGyroZ1 = 0;
long calibGyroX2 = 0, calibGyroY2 = 0, calibGyroZ2 = 0;
float calibMagX = 0, calibMagY = 0;
int calibGyroCount = 0;
int calibMagCount = 0;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// ==========================================
// INTERRUPTS & HARDWARE HELPERS
// ==========================================
void IRAM_ATTR isrCalibrate() { buttonPressed = true; }

void writeRegisterIMU(int csPin, uint8_t reg, uint8_t val) {
  digitalWrite(csPin, LOW); SPI.transfer(reg & 0x7F); SPI.transfer(val); digitalWrite(csPin, HIGH);
}

void readIMUDataBurst(int csPin, volatile int16_t &ax, volatile int16_t &ay, volatile int16_t &az, volatile int16_t &gx, volatile int16_t &gy, volatile int16_t &gz) {
  digitalWrite(csPin, LOW);
  SPI.transfer(0x1F | 0x80); 
  ax = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00); ay = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  az = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00); gx = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gy = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00); gz = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
}

void writeRegisterHaptic(uint8_t reg, uint8_t val) {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
}

void triggerHapticPulse() { writeRegisterHaptic(0x04, 1); writeRegisterHaptic(0x05, 0); writeRegisterHaptic(0x0C, 1); }
void triggerLongHapticPulse() { writeRegisterHaptic(0x04, 14); writeRegisterHaptic(0x05, 14); writeRegisterHaptic(0x06, 0); writeRegisterHaptic(0x0C, 1); }

void triggerHapticHit(int velocityBucket) {
  uint8_t effect = 1;
  switch(velocityBucket) {
    case 8: effect = 1; break; case 7: effect = 2; break; case 6: effect = 3; break; case 5: effect = 4; break;
    case 4: effect = 5; break; case 3: effect = 6; break; case 2: effect = 7; break; case 1: effect = 8; break;
  }
  writeRegisterHaptic(0x04, effect); writeRegisterHaptic(0x05, 0x00); writeRegisterHaptic(0x0C, 1);      
}

void initMagnetometer() { Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x20); Wire.endTransmission(); }

bool readMagnetometer(float &mx, float &my, float &mz) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))) {
    Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x01); Wire.endTransmission();
    delayMicroseconds(1000); 
    Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x00); Wire.endTransmission();
    Wire.requestFrom(MMC5983_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      uint32_t rawX = (Wire.read() << 10) | (Wire.read() << 2);
      uint32_t rawY = (Wire.read() << 10) | (Wire.read() << 2);
      uint32_t rawZ = (Wire.read() << 10) | (Wire.read() << 2);
      mx = (float)((long)rawX - 131072); my = (float)((long)rawY - 131072); mz = (float)((long)rawZ - 131072);
      xSemaphoreGive(i2cMutex); return true;
    }
    xSemaphoreGive(i2cMutex);
  } return false;
}

void processMagnetometerCalibration() {
  if (magSampleCount < 50) return;
  float minX = 99999, maxX = -99999, minY = 99999, maxY = -99999, minZ = 99999, maxZ = -99999;
  for (int i = 0; i < magSampleCount; i++) {
    if (magBuffer[i].x < minX) minX = magBuffer[i].x; if (magBuffer[i].x > maxX) maxX = magBuffer[i].x;
    if (magBuffer[i].y < minY) minY = magBuffer[i].y; if (magBuffer[i].y > maxY) maxY = magBuffer[i].y;
    if (magBuffer[i].z < minZ) minZ = magBuffer[i].z; if (magBuffer[i].z > maxZ) maxZ = magBuffer[i].z;
  }
  magBiasX = (maxX + minX) / 2.0; magBiasY = (maxY + minY) / 2.0; magBiasZ = (maxZ + minZ) / 2.0;
  float chordX = (maxX - minX) / 2.0; float chordY = (maxY - minY) / 2.0; float chordZ = (maxZ - minZ) / 2.0;
  float avgChord = (chordX + chordY + chordZ) / 3.0;
  magScaleX = (chordX > 0) ? (avgChord / chordX) : 1.0; magScaleY = (chordY > 0) ? (avgChord / chordY) : 1.0; magScaleZ = (chordZ > 0) ? (avgChord / chordZ) : 1.0;

  prefs.begin("drum_cal", false);
  prefs.putFloat("magBiasX", magBiasX); prefs.putFloat("magBiasY", magBiasY); prefs.putFloat("magBiasZ", magBiasZ);
  prefs.putFloat("magScaleX", magScaleX); prefs.putFloat("magScaleY", magScaleY); prefs.putFloat("magScaleZ", magScaleZ);
  prefs.end();
  Serial.println("\n✅ [MAG CALIBRATION SAVED TO FLASH]");
}

void autoCalibrateHaptic() {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x01); Wire.write(0x07); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1A); Wire.write(0xB6); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x16); Wire.write(0x53); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x17); Wire.write(0xA4); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1B); Wire.write(0x93); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1C); Wire.write(0x25); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x0C); Wire.write(0x01); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
  delay(1000); 
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
}

// ==========================================
// CORE 1: FAST PHYSICS TASK (Untouched Math!)
// ==========================================
void PhysicsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool impactLatched = false;
  float latchedPitch = 0.0;
  float latchedYaw = 0.0;

  for(;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    readIMUDataBurst(IMU1_CS, cur_ax1, cur_ay1, cur_az1, cur_gx1, cur_gy1, cur_gz1);
    readIMUDataBurst(IMU2_CS, cur_ax2, cur_ay2, cur_az2, cur_gx2, cur_gy2, cur_gz2);

    if (sysState != STATE_NORMAL) continue;
    float dt = 0.001; 
    
    float gx_rad = (cur_gx2 - gyroBiasX2) * 0.001065264f;
    float gy_rad = (cur_gy2 - gyroBiasY2) * 0.001065264f;
    float gz_rad = (cur_gz2 - gyroBiasZ2) * 0.001065264f;

    float dq0 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * dt;
    float dq1 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * dt;
    float dq2 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * dt;
    float dq3 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * dt;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    float curPitch = -asin(2.0f * (q0*q2 - q3*q1)) * 57.2957f;
    float curYaw   = -atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.2957f;

    float finalPitch = curPitch - targetPitchOffset;
    float finalYaw   = curYaw - targetYawOffset;
    while (finalYaw > 180.0) finalYaw -= 360.0;
    while (finalYaw < -180.0) finalYaw += 360.0;

    int16_t current_gyro_y = cur_gy2;
    if (stickState == STATE_IDLE) {
      if (current_gyro_y > SWING_START_THRESHOLD) { 
        stickState = STATE_SWINGING;
        peak_swing_velocity = current_gyro_y; 
        impactLatched = false;
      }
    } 
    else if (stickState == STATE_SWINGING) {
      if (current_gyro_y > peak_swing_velocity) { peak_swing_velocity = current_gyro_y; }

      if (!impactLatched && current_gyro_y < (peak_swing_velocity - 800)) {
        latchedPitch = finalPitch; latchedYaw = finalYaw; impactLatched = true;
      }

      if (current_gyro_y < (peak_swing_velocity - HIT_DECEL_THRESHOLD)) {
        uint8_t drumId = 0;
        if (latchedYaw >= -90.0 && latchedYaw <= 90.0) {
          bool isTopRow = (latchedPitch >= 25.0);
          if (latchedYaw < -30.0) { drumId = isTopRow ? 1 : 6; } 
          else if (latchedYaw >= -30.0 && latchedYaw < 0.0) { drumId = isTopRow ? 3 : 2; } 
          else if (latchedYaw >= 0.0 && latchedYaw <= 30.0) { drumId = isTopRow ? 4 : 2; } 
          else if (latchedYaw > 30.0) { drumId = isTopRow ? 5 : 7; }
        }
        
        if (drumId != 0) {
          int mapped_vel = constrain(map(peak_swing_velocity, 4000, 25000, 1, 8), 1, 8);
          triggerHapticHit(mapped_vel);
          uint8_t udpVel = constrain(map(mapped_vel, 1, 8, 1, 6), 1, 6);
          HitPayload payload = {STICK_ID, drumId, udpVel};
          
          // Send to Network Task via Queue instantly
          xQueueSend(hitQueue, &payload, 0);
        }
        stickState = STATE_REFRACTORY; hitTimer = millis();
      }
    } 
    else if (stickState == STATE_REFRACTORY) {
      if (millis() - hitTimer >= 80) { 
        if (current_gyro_y < 2000) { stickState = STATE_IDLE; peak_swing_velocity = 0; }
      }
    }
  }
}

// ==========================================
// CORE 0: NETWORK & STATE MACHINE TASK
// ==========================================
void NetworkTask(void *pvParameters) {
  // --- Initialize ESP-NOW ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Init Failed!"); }
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add Hub peer"); }

  autoCalibrateHaptic(); 
  stateTimer = millis();
  sysState = STATE_BOOT_WAIT;

  TickType_t lastMagRead = xTaskGetTickCount();
  for(;;) {
    unsigned long currentMillis = millis();

    HitPayload outgoingHit;
    // OPTIMIZATION: Block for up to 1ms waiting for a hit.
    // This replaces vTaskDelay(1) and eliminates up to 1ms of delay!
    if (xQueueReceive(hitQueue, &outgoingHit, pdMS_TO_TICKS(1)) == pdPASS) {
      esp_now_send(hubAddress, (uint8_t *) &outgoingHit, sizeof(HitPayload));
    }

    if (buttonPressed && sysState == STATE_NORMAL) {
      buttonPressed = false; sysState = STATE_CAL_WAIT; stateTimer = currentMillis; digitalWrite(PIN_STATUS_LED, HIGH);
    } else { buttonPressed = false; }

    if (sysState == STATE_BOOT_WAIT) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      if (currentMillis - stateTimer >= 2000) {
        sysState = STATE_BOOT_QUICK_SNAP; stateTimer = currentMillis;
        calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0; calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
        calibMagX = 0; calibMagY = 0; calibGyroCount = 0; calibMagCount = 0;
      }
    } 
    else if (sysState == STATE_BOOT_QUICK_SNAP) {
      if (currentMillis - lastLedToggle >= 100) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      calibGyroX1 += cur_gx1; calibGyroY1 += cur_gy1; calibGyroZ1 += cur_gz1; calibGyroX2 += cur_gx2; calibGyroY2 += cur_gy2; calibGyroZ2 += cur_gz2; calibGyroCount++;

      float mx, my, mz;
      if (readMagnetometer(mx, my, mz)) { calibMagX += (mx - magBiasX) * magScaleX; calibMagY += (my - magBiasY) * magScaleY; calibMagCount++; }

      if (currentMillis - stateTimer >= 1500) {
        gyroBiasX1 = (float)calibGyroX1 / calibGyroCount; gyroBiasY1 = (float)calibGyroY1 / calibGyroCount; gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
        gyroBiasX2 = (float)calibGyroX2 / calibGyroCount; gyroBiasY2 = (float)calibGyroY2 / calibGyroCount; gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;
        
        float ax = (cur_ax1 + cur_ax2) / 2.0; float ay = (cur_ay1 + cur_ay2) / 2.0; float az = (cur_az1 + cur_az2) / 2.0;
        pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; targetPitchOffset = pitch;
        float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1); float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
        yaw = atan2(avgMagY, avgMagX) * 57.2957; targetYawOffset = yaw;
        triggerHapticPulse(); sysState = STATE_NORMAL;
      }
    }
    else if (sysState == STATE_CAL_WAIT) {
      if (currentMillis - stateTimer >= 2000) { sysState = STATE_CAL_FIG8; stateTimer = currentMillis; magSampleCount = 0; triggerLongHapticPulse(); }
    }
    else if (sysState == STATE_CAL_FIG8) {
      if (currentMillis - lastMagSampleTime >= 50 && magSampleCount < MAX_MAG_SAMPLES) {
        lastMagSampleTime = currentMillis; float mx, my, mz;
        if (readMagnetometer(mx, my, mz)) magBuffer[magSampleCount++] = {mx, my, mz};
      }
      if (currentMillis - stateTimer >= 15000) {
        processMagnetometerCalibration(); triggerHapticPulse(); sysState = STATE_CAL_WAIT_SNARE; stateTimer = currentMillis; digitalWrite(PIN_STATUS_LED, LOW); 
      }
    }
    else if (sysState == STATE_CAL_WAIT_SNARE) {
      if (currentMillis - stateTimer >= 4000) {
        sysState = STATE_CAL_SNARE; stateTimer = currentMillis;
        calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0; calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
        calibMagX = 0; calibMagY = 0; calibGyroCount = 0; calibMagCount = 0;
      }
    }
    else if (sysState == STATE_CAL_SNARE) {
      if (currentMillis - lastLedToggle >= 100) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      calibGyroX1 += cur_gx1; calibGyroY1 += cur_gy1; calibGyroZ1 += cur_gz1; calibGyroX2 += cur_gx2; calibGyroY2 += cur_gy2; calibGyroZ2 += cur_gz2; calibGyroCount++;

      float mx, my, mz;
      if (readMagnetometer(mx, my, mz)) { calibMagX += (mx - magBiasX) * magScaleX; calibMagY += (my - magBiasY) * magScaleY; calibMagCount++; }

      if (currentMillis - stateTimer >= 5000) {
        gyroBiasX1 = (float)calibGyroX1 / calibGyroCount; gyroBiasY1 = (float)calibGyroY1 / calibGyroCount; gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
        gyroBiasX2 = (float)calibGyroX2 / calibGyroCount; gyroBiasY2 = (float)calibGyroY2 / calibGyroCount; gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;
        
        float ax = (cur_ax1 + cur_ax2) / 2.0; float ay = (cur_ay1 + cur_ay2) / 2.0; float az = (cur_az1 + cur_az2) / 2.0;
        pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; targetPitchOffset = pitch;
        float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1); float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
        yaw = atan2(avgMagY, avgMagX) * 57.2957; targetYawOffset = yaw;
        triggerHapticPulse(); sysState = STATE_NORMAL;
      }
    }
    else if (sysState == STATE_NORMAL) {
      if (currentMillis - lastLedToggle >= 1000) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      if (xTaskGetTickCount() - lastMagRead >= pdMS_TO_TICKS(20)) {
        lastMagRead = xTaskGetTickCount(); float mX, mY, mZ;
        if (readMagnetometer(mX, mY, mZ)) {
          float cX = (mX - magBiasX) * magScaleX; float cY = (mY - magBiasY) * magScaleY;
          sharedMagHeading = atan2(cY, cX) * 57.2957;
        }
      }
    }
  }
}

// ==========================================
// ARDUINO SETUP
// ==========================================
void setup() {
  pinMode(PMIC_KILL, OUTPUT); digitalWrite(PMIC_KILL, HIGH);
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, HIGH);
  pinMode(IMU1_CS, OUTPUT); digitalWrite(IMU1_CS, HIGH);
  pinMode(IMU2_CS, OUTPUT); digitalWrite(IMU2_CS, HIGH);

  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  pinMode(PMIC_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PMIC_INT), isrCalibrate, FALLING);

  i2cMutex = xSemaphoreCreateMutex();
  hitQueue = xQueueCreate(10, sizeof(HitPayload));

  writeRegisterHaptic(0x01, 0x00); writeRegisterHaptic(0x03, 0x06); 
  writeRegisterHaptic(0x16, 0x56); writeRegisterHaptic(0x17, 0xFF);
  writeRegisterHaptic(0x1A, 0xB6); writeRegisterHaptic(0x1B, 0x93);
  writeRegisterHaptic(0x1C, 0x75); writeRegisterHaptic(0x1D, 0x80);
  
  initMagnetometer();
  writeRegisterIMU(IMU1_CS, 0x4E, 0x0F); writeRegisterIMU(IMU2_CS, 0x4E, 0x0F);
  delay(100);

  prefs.begin("drum_cal", true);
  magBiasX = prefs.getFloat("magBiasX", 0.0); magBiasY = prefs.getFloat("magBiasY", 0.0); magBiasZ = prefs.getFloat("magBiasZ", 0.0);
  magScaleX = prefs.getFloat("magScaleX", 1.0); magScaleY = prefs.getFloat("magScaleY", 1.0); magScaleZ = prefs.getFloat("magScaleZ", 1.0);
  prefs.end();

  xTaskCreatePinnedToCore(NetworkTask, "NetTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(PhysicsTask, "PhysTask", 8192, NULL, 2, NULL, 1); 
}

void loop() { vTaskDelete(NULL); }