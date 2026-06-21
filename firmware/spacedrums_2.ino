#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ==========================================
// CONFIGURATION
// ==========================================
const char* WIFI_SSID = "---";
const char* WIFI_PASS = "---";
const char* UDP_TARGET_IP = "255.255.255.255"; 
const int   UDP_PORT      = 5005;

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
QueueHandle_t udpQueue;
WiFiUDP udp;

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

// Shared IMU Data (Written by Core 1, Read by Core 0 for Calibration)
volatile int16_t cur_ax1, cur_ay1, cur_az1, cur_gx1, cur_gy1, cur_gz1;
volatile int16_t cur_ax2, cur_ay2, cur_az2, cur_gx2, cur_gy2, cur_gz2;

// Hit Detection
enum DrumState { STATE_IDLE, STATE_SWINGING, STATE_REFRACTORY };
DrumState stickState = STATE_IDLE;
const int16_t SWING_START_THRESHOLD = 4000;
const int16_t HIT_DECEL_THRESHOLD   = 3000;
const unsigned long REFRACTORY_TIME_MS = 80; 
int16_t peak_swing_velocity = 0; 
unsigned long hitTimer = 0;

// Orientation
float pitch = 0.0, yaw = 0.0;
float targetYawOffset = 0.0, targetPitchOffset = 0.0;
volatile float sharedMagHeading = 0.0; 

// Independent IMU Biases
float gyroBiasX1 = 0, gyroBiasY1 = 0, gyroBiasZ1 = 0;
float gyroBiasX2 = 0, gyroBiasY2 = 0, gyroBiasZ2 = 0;
float magBiasX = 0, magBiasY = 0, magBiasZ = 0;
float magScaleX = 1.0, magScaleY = 1.0, magScaleZ = 1.0;

// Figure-8 Buffer
const int MAX_MAG_SAMPLES = 300;
struct MagSample { float x, y, z; };
MagSample magBuffer[MAX_MAG_SAMPLES];
int magSampleCount = 0;
unsigned long lastMagSampleTime = 0;

// Calibration Accumulation Buffers
long calibGyroX1 = 0, calibGyroY1 = 0, calibGyroZ1 = 0;
long calibGyroX2 = 0, calibGyroY2 = 0, calibGyroZ2 = 0;
float calibMagX = 0, calibMagY = 0;
int calibGyroCount = 0;
int calibMagCount = 0;

// ==========================================
// GLOBALS FOR QUATERNION MATH
// ==========================================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternion [w, x, y, z]

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

// I2C Mutex Protected Hardware Functions
void writeRegisterHaptic(uint8_t reg, uint8_t val) {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
}

void triggerHapticPulse() {
  writeRegisterHaptic(0x04, 1); writeRegisterHaptic(0x05, 0); writeRegisterHaptic(0x0C, 1); 
}

void triggerLongHapticPulse() {
  writeRegisterHaptic(0x04, 14); writeRegisterHaptic(0x05, 14); writeRegisterHaptic(0x06, 0); writeRegisterHaptic(0x0C, 1); 
}

void triggerHapticHit(int velocityBucket) {
  uint8_t effect = 1; 
  switch(velocityBucket) {
    case 8: effect = 1; break;  // 100% Strong Click
    case 7: effect = 2; break;  // 80% Strong Click
    case 6: effect = 3; break;  // 60% Strong Click
    case 5: effect = 4; break;  // 40% Strong Click
    case 4: effect = 5; break;  // 20% Strong Click
    case 3: effect = 6; break;  // 100% Sharp Tick
    case 2: effect = 7; break;  // 80% Sharp Tick
    case 1: effect = 8; break;  // 60% Sharp Tick
  }
  writeRegisterHaptic(0x04, effect); // Slot 1: Play effect
  writeRegisterHaptic(0x05, 0x00);   // Slot 2: TERMINATE SEQUENCE (Prevents double playing)
  writeRegisterHaptic(0x0C, 1);      // GO
}

void initMagnetometer() {
  Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x20); Wire.endTransmission();
}

bool readMagnetometer(float &mx, float &my, float &mz) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))) {
    Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x01); Wire.endTransmission();
    delayMicroseconds(1000); // Safe in Core 0 or during Init
    Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x00); Wire.endTransmission();
    Wire.requestFrom(MMC5983_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      uint32_t rawX = (Wire.read() << 10) | (Wire.read() << 2);
      uint32_t rawY = (Wire.read() << 10) | (Wire.read() << 2);
      uint32_t rawZ = (Wire.read() << 10) | (Wire.read() << 2);
      mx = (float)((long)rawX - 131072); my = (float)((long)rawY - 131072); mz = (float)((long)rawZ - 131072);
      xSemaphoreGive(i2cMutex);
      return true;
    }
    xSemaphoreGive(i2cMutex);
  }
  return false;
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
  magScaleX = (chordX > 0) ? (avgChord / chordX) : 1.0;
  magScaleY = (chordY > 0) ? (avgChord / chordY) : 1.0;
  magScaleZ = (chordZ > 0) ? (avgChord / chordZ) : 1.0;

  prefs.begin("drum_cal", false);
  prefs.putFloat("magBiasX", magBiasX); prefs.putFloat("magBiasY", magBiasY); prefs.putFloat("magBiasZ", magBiasZ);
  prefs.putFloat("magScaleX", magScaleX); prefs.putFloat("magScaleY", magScaleY); prefs.putFloat("magScaleZ", magScaleZ);
  prefs.end();
  Serial.println("\n✅ [MAG CALIBRATION SAVED TO FLASH]");
}

void autoCalibrateHaptic() {
  Serial.println("⚙️ [HAPTIC] Running DRV2605L Auto-Calibration (Wait 1s)...");
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    // Mode 7: Auto Calibration
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x01); Wire.write(0x07); Wire.endTransmission();
    
    // Feedback Control: LRA mode (bit 7=1), Brake factor (4x), Loop Gain (High)
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1A); Wire.write(0xB6); Wire.endTransmission();
    
    // Rated Voltage & Overdrive Clamp (Optimized for standard 2Vrms 3.2Vpeak LRAs)
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x16); Wire.write(0x53); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x17); Wire.write(0xA4); Wire.endTransmission();
    
    // Control 1-4 (Drive time tuning)
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1B); Wire.write(0x93); Wire.endTransmission();
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x1C); Wire.write(0x25); Wire.endTransmission();
    
    // Set GO Bit!
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x0C); Wire.write(0x01); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
  
  delay(1000); // The motor will physically vibrate and sweep frequencies here
  
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    // Read status (Bit 3 indicates failure)
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x00); Wire.endTransmission();
    Wire.requestFrom((uint8_t)DRV2605_ADDR, (uint8_t)1);
    uint8_t status = Wire.read();
    if (status & 0x08) Serial.println("⚠️ [HAPTIC] Calibration Failed/Interrupted!");
    else Serial.println("✅ [HAPTIC] Calibration Success! (Closed-loop tracking enabled)");
    
    // Return to Normal operation mode (Mode 0)
    Wire.beginTransmission(DRV2605_ADDR); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
}


// ==========================================
// CORE 1: FAST PHYSICS TASK (1000 Hz)
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
    
    // ==========================================
    // 1. QUATERNION ORIENTATION (Zero Gimbal Lock)
    // ==========================================
    // We strictly use IMU2 (Hand) to avoid the tip whip distortion
    float gx_rad = (cur_gx2 - gyroBiasX2) * 0.001065264f; // Convert raw to rad/s
    float gy_rad = (cur_gy2 - gyroBiasY2) * 0.001065264f;
    float gz_rad = (cur_gz2 - gyroBiasZ2) * 0.001065264f;

    // Pure Kinematic Integration
    float dq0 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * dt;
    float dq1 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * dt;
    float dq2 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * dt;
    float dq3 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * dt;

    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    // Normalize Quaternion
    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // Extract mathematically perfect Euler Angles for drum mapping
    float curPitch = -asin(2.0f * (q0*q2 - q3*q1)) * 57.2957f; 
    float curYaw   = -atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.2957f;

    // Apply offsets
    float finalPitch = curPitch - targetPitchOffset;
    float finalYaw   = curYaw - targetYawOffset;
    while (finalYaw > 180.0) finalYaw -= 360.0;
    while (finalYaw < -180.0) finalYaw += 360.0;

    // ==========================================
    // 2. KINEMATIC HIT DETECTION (Impact-Onset)
    // ==========================================
    int16_t current_gyro_y = cur_gy2; 
    
    if (stickState == STATE_IDLE) {
      if (current_gyro_y > SWING_START_THRESHOLD) { 
        stickState = STATE_SWINGING;
        peak_swing_velocity = current_gyro_y; 
        impactLatched = false;
      }
    } 
    else if (stickState == STATE_SWINGING) {
      
      // Track maximum speed
      if (current_gyro_y > peak_swing_velocity) {
        peak_swing_velocity = current_gyro_y;
      }

      // 🎯 THE ONSET LATCH
      // We grab the angles the *millisecond* the stick begins to decelerate.
      // The stick has reached the intended target, but the physical "whip" hasn't happened yet.
      if (!impactLatched && current_gyro_y < (peak_swing_velocity - 800)) {
        latchedPitch = finalPitch;
        latchedYaw = finalYaw;
        impactLatched = true;
      }

      // 💥 THE FULL IMPACT (Triggering the Sound)
      // We wait for the violent stop to confirm it wasn't a fake swing.
      if (current_gyro_y < (peak_swing_velocity - HIT_DECEL_THRESHOLD)) {
        
        uint8_t drumId = 0;

        // We use the LATCHED angles, ignoring whatever chaotic angle the stick is currently at
        if (latchedYaw >= -90.0 && latchedYaw <= 90.0) {
          
          // 25.0 degrees is the true physical bisector between a cymbal hit and a snare hit
          bool isTopRow = (latchedPitch >= 25.0);

          if (latchedYaw < -30.0) {
              drumId = isTopRow ? 1 /* Crash */ : 6 /* Hi-Hat */;
          } else if (latchedYaw >= -30.0 && latchedYaw < 0.0) {
              drumId = isTopRow ? 3 /* Tom 1 */ : 2 /* Snare */;
          } else if (latchedYaw >= 0.0 && latchedYaw <= 30.0) {
              drumId = isTopRow ? 4 /* Tom 2 */ : 2 /* Snare */;
          } else if (latchedYaw > 30.0) {
              drumId = isTopRow ? 5 /* Ride  */ : 7 /* Floor Tom */;
          }
        }
        
        if (drumId != 0) {
          int mapped_vel = constrain(map(peak_swing_velocity, 4000, 25000, 1, 8), 1, 8);
          triggerHapticHit(mapped_vel);
          
          uint8_t udpVel = constrain(map(mapped_vel, 1, 8, 1, 6), 1, 6);
          HitPayload payload = {STICK_ID, drumId, udpVel};
          xQueueSend(udpQueue, &payload, 0); 
        }
        
        stickState = STATE_REFRACTORY;
        hitTimer = millis();
      }
    } 
    else if (stickState == STATE_REFRACTORY) {
      if (millis() - hitTimer >= 80) { 
        if (current_gyro_y < 2000) {
          stickState = STATE_IDLE;
          peak_swing_velocity = 0;
        }
      }
    }
  }
}


// ==========================================
// CORE 0: NETWORK & STATE MACHINE TASK
// ==========================================
void NetworkTask(void *pvParameters) {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(500));

  autoCalibrateHaptic(); // Runs the actual DRV sweep!
  stateTimer = millis();
  sysState = STATE_BOOT_WAIT; 
  Serial.println("\n🚀 [SYSTEM BOOT] Move to Snare Position. (Wait 2s...)");

  TickType_t lastMagRead = xTaskGetTickCount();

  for(;;) {
    unsigned long currentMillis = millis();

    // 1. Process UDP Network Queue
    HitPayload outgoingHit;
    if (xQueueReceive(udpQueue, &outgoingHit, 0) == pdPASS) {
      uint8_t packet[3] = {outgoingHit.stick_id, outgoingHit.drum_id, outgoingHit.velocity};
      udp.beginPacket(UDP_TARGET_IP, UDP_PORT); udp.write(packet, 3); udp.endPacket();
    }

    // 2. Button Override Logic
    if (buttonPressed && sysState == STATE_NORMAL) {
      buttonPressed = false;
      sysState = STATE_CAL_WAIT;
      stateTimer = currentMillis;
      digitalWrite(PIN_STATUS_LED, HIGH);
      Serial.println("\n⏳ [FULL CALIBRATION TRIGGERED] Wait 2 seconds...");
    } else { buttonPressed = false; }

    // 3. FULL STATE MACHINE
    if (sysState == STATE_BOOT_WAIT) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      if (currentMillis - stateTimer >= 2000) {
        sysState = STATE_BOOT_QUICK_SNAP; stateTimer = currentMillis;
        calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0; calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
        calibMagX = 0; calibMagY = 0; calibGyroCount = 0; calibMagCount = 0;
        Serial.println("⚡ [QUICK SNAP] Averaging gyros... (Hold still 1.5s)");
      }
    } 
    else if (sysState == STATE_BOOT_QUICK_SNAP) {
      if (currentMillis - lastLedToggle >= 100) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      
      calibGyroX1 += cur_gx1; calibGyroY1 += cur_gy1; calibGyroZ1 += cur_gz1;
      calibGyroX2 += cur_gx2; calibGyroY2 += cur_gy2; calibGyroZ2 += cur_gz2; calibGyroCount++;

      float mx, my, mz;
      if (readMagnetometer(mx, my, mz)) {
        calibMagX += (mx - magBiasX) * magScaleX; calibMagY += (my - magBiasY) * magScaleY; calibMagCount++;
      }

      if (currentMillis - stateTimer >= 1500) {
        gyroBiasX1 = (float)calibGyroX1 / calibGyroCount; gyroBiasY1 = (float)calibGyroY1 / calibGyroCount; gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
        gyroBiasX2 = (float)calibGyroX2 / calibGyroCount; gyroBiasY2 = (float)calibGyroY2 / calibGyroCount; gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;
        
        float ax = (cur_ax1 + cur_ax2) / 2.0; float ay = (cur_ay1 + cur_ay2) / 2.0; float az = (cur_az1 + cur_az2) / 2.0;
        pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; targetPitchOffset = pitch; 
        
        float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1);
        float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
        yaw = atan2(avgMagY, avgMagX) * 57.2957; targetYawOffset = yaw;

        triggerHapticPulse();
        sysState = STATE_NORMAL;
        Serial.println("🥁 [READY] Boot calibration complete! Start playing.");
      }
    }
    else if (sysState == STATE_CAL_WAIT) {
      if (currentMillis - stateTimer >= 2000) {
        sysState = STATE_CAL_FIG8; stateTimer = currentMillis; magSampleCount = 0;
        triggerLongHapticPulse(); Serial.println("♾️ [FIGURE-8] Start waving! (15 Seconds)");
      }
    }
    else if (sysState == STATE_CAL_FIG8) {
      if (currentMillis - lastMagSampleTime >= 50 && magSampleCount < MAX_MAG_SAMPLES) {
        lastMagSampleTime = currentMillis;
        float mx, my, mz;
        if (readMagnetometer(mx, my, mz)) magBuffer[magSampleCount++] = {mx, my, mz};
      }
      if (currentMillis - stateTimer >= 15000) {
        processMagnetometerCalibration(); 
        triggerHapticPulse();
        sysState = STATE_CAL_WAIT_SNARE; stateTimer = currentMillis; digitalWrite(PIN_STATUS_LED, LOW); 
        Serial.println("🛑 [STOP] Move to Snare Position and hold still... (4 Seconds)");
      }
    }
    else if (sysState == STATE_CAL_WAIT_SNARE) {
      if (currentMillis - stateTimer >= 4000) {
        sysState = STATE_CAL_SNARE; stateTimer = currentMillis;
        calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0; calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
        calibMagX = 0; calibMagY = 0; calibGyroCount = 0; calibMagCount = 0;
        Serial.println("🎯 [SNARE LOCK] Averaging independent IMU biases... (5 Seconds)");
      }
    }
    else if (sysState == STATE_CAL_SNARE) {
      if (currentMillis - lastLedToggle >= 100) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      
      calibGyroX1 += cur_gx1; calibGyroY1 += cur_gy1; calibGyroZ1 += cur_gz1;
      calibGyroX2 += cur_gx2; calibGyroY2 += cur_gy2; calibGyroZ2 += cur_gz2; calibGyroCount++;

      float mx, my, mz;
      if (readMagnetometer(mx, my, mz)) {
        calibMagX += (mx - magBiasX) * magScaleX; calibMagY += (my - magBiasY) * magScaleY; calibMagCount++;
      }

      if (currentMillis - stateTimer >= 5000) {
        gyroBiasX1 = (float)calibGyroX1 / calibGyroCount; gyroBiasY1 = (float)calibGyroY1 / calibGyroCount; gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
        gyroBiasX2 = (float)calibGyroX2 / calibGyroCount; gyroBiasY2 = (float)calibGyroY2 / calibGyroCount; gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;
        
        float ax = (cur_ax1 + cur_ax2) / 2.0; float ay = (cur_ay1 + cur_ay2) / 2.0; float az = (cur_az1 + cur_az2) / 2.0;
        pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; targetPitchOffset = pitch; 
        
        float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1);
        float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
        yaw = atan2(avgMagY, avgMagX) * 57.2957; targetYawOffset = yaw;

        triggerHapticPulse(); sysState = STATE_NORMAL;
        Serial.println("🥁 [READY] Deep calibration mapped! System Tracking.");
      }
    }
    else if (sysState == STATE_NORMAL) {
      // 0.5 Hz LED Heartbeat
      if (currentMillis - lastLedToggle >= 1000) { lastLedToggle = currentMillis; ledState = !ledState; digitalWrite(PIN_STATUS_LED, ledState); }
      
      // Slow background Mag updates for drift correction
      if (xTaskGetTickCount() - lastMagRead >= pdMS_TO_TICKS(20)) {
        lastMagRead = xTaskGetTickCount();
        float mX, mY, mZ;
        if (readMagnetometer(mX, mY, mZ)) {
          float cX = (mX - magBiasX) * magScaleX; float cY = (mY - magBiasY) * magScaleY;
          sharedMagHeading = atan2(cY, cX) * 57.2957;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // Yield to watchdog
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
  udpQueue = xQueueCreate(10, sizeof(HitPayload));

  // Init Busses & Hardware (Mutex not needed yet, tasks haven't started)
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

  // Boot Tasks
  xTaskCreatePinnedToCore(NetworkTask, "NetTask", 8192, NULL, 1, NULL, 0); 
  xTaskCreatePinnedToCore(PhysicsTask, "PhysTask", 8192, NULL, 2, NULL, 1); 
}

void loop() {
  vTaskDelete(NULL); 
}
