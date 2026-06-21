// hold at snare for 2 secs immediately after power up for calibration
// for detailed calib, press power button anf do figure of 8 followed by hold at snare

#include <SPI.h>
#include <Wire.h>
#include <Preferences.h> // ESP32 Non-Volatile Storage Library

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

// --- PREFERENCES STORAGE ---
Preferences prefs;

// --- STATE MACHINE ---
enum SystemState { 
  STATE_BOOT_WAIT,        // 2-second wait on power-on
  STATE_BOOT_QUICK_SNAP,  // 1.5-second fast gyro/snare lock
  STATE_NORMAL,           // Active drum tracking
  STATE_CAL_WAIT,         // 2-second wait after button press
  STATE_CAL_FIG8,         // 15-second Mag calibration
  STATE_CAL_WAIT_SNARE,   // 4-second wait to get to snare
  STATE_CAL_SNARE         // 5-second deep gyro/snare lock
};
volatile SystemState sysState = STATE_BOOT_WAIT;
volatile bool buttonPressed = false;

unsigned long stateTimer = 0;
unsigned long lastLedToggle = 0;
bool ledState = false;

// --- SENSOR DATA & OFFSETS ---
float pitch = 0.0, yaw = 0.0;
float targetYawOffset = 0.0, targetPitchOffset = 0.0;
unsigned long lastMicros = 0;

// Independent IMU Biases
float gyroBiasX1 = 0, gyroBiasY1 = 0, gyroBiasZ1 = 0;
float gyroBiasX2 = 0, gyroBiasY2 = 0, gyroBiasZ2 = 0;

// Magnetometer Calibration Variables
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

// --- INTERRUPT ---
void IRAM_ATTR isrCalibrate() {
  buttonPressed = true;
}

// --- HARDWARE DRIVERS ---
void writeRegisterIMU(int csPin, uint8_t reg, uint8_t val) {
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x7F); SPI.transfer(val);
  digitalWrite(csPin, HIGH);
}

void readIMUDataBurst(int csPin, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  digitalWrite(csPin, LOW);
  SPI.transfer(0x1F | 0x80); // Accel X High Byte Register
  ax = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  ay = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  az = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gx = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gy = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gz = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
}

void writeRegisterHaptic(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV2605_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

void triggerHapticPulse() {
  // Short, sharp click
  writeRegisterHaptic(0x04, 1); 
  writeRegisterHaptic(0x05, 0); 
  writeRegisterHaptic(0x0C, 1); 
}

void triggerLongHapticPulse() {
  // Long buzz sequence to signify bootup/calibration start
  writeRegisterHaptic(0x04, 14); // Strong Buzz 1
  writeRegisterHaptic(0x05, 14); // Strong Buzz 1 (queued twice for length)
  writeRegisterHaptic(0x06, 0); 
  writeRegisterHaptic(0x0C, 1); 
}

void initMagnetometer() {
  Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x20); Wire.endTransmission();
}

bool readMagnetometer(float &mx, float &my, float &mz) {
  Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x09); Wire.write(0x01); Wire.endTransmission();
  delayMicroseconds(1000);
  Wire.beginTransmission(MMC5983_ADDR); Wire.write(0x00); Wire.endTransmission();
  Wire.requestFrom(MMC5983_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint32_t rawX = (Wire.read() << 10) | (Wire.read() << 2);
    uint32_t rawY = (Wire.read() << 10) | (Wire.read() << 2);
    uint32_t rawZ = (Wire.read() << 10) | (Wire.read() << 2);
    mx = (float)((long)rawX - 131072);
    my = (float)((long)rawY - 131072);
    mz = (float)((long)rawZ - 131072);
    return true;
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

  magBiasX = (maxX + minX) / 2.0;
  magBiasY = (maxY + minY) / 2.0;
  magBiasZ = (maxZ + minZ) / 2.0;

  float chordX = (maxX - minX) / 2.0;
  float chordY = (maxY - minY) / 2.0;
  float chordZ = (maxZ - minZ) / 2.0;

  float avgChord = (chordX + chordY + chordZ) / 3.0;
  magScaleX = (chordX > 0) ? (avgChord / chordX) : 1.0;
  magScaleY = (chordY > 0) ? (avgChord / chordY) : 1.0;
  magScaleZ = (chordZ > 0) ? (avgChord / chordZ) : 1.0;

  // --- SAVE CALIBRATION PERMANENTLY ---
  prefs.begin("drum_cal", false);
  prefs.putFloat("magBiasX", magBiasX);
  prefs.putFloat("magBiasY", magBiasY);
  prefs.putFloat("magBiasZ", magBiasZ);
  prefs.putFloat("magScaleX", magScaleX);
  prefs.putFloat("magScaleY", magScaleY);
  prefs.putFloat("magScaleZ", magScaleZ);
  prefs.end();

  Serial.println("\n✅ [MAG CALIBRATION SAVED TO FLASH]");
}

void setup() {
  pinMode(PMIC_KILL, OUTPUT); digitalWrite(PMIC_KILL, HIGH);
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, HIGH);

  pinMode(IMU1_CS, OUTPUT); pinMode(IMU2_CS, OUTPUT);
  digitalWrite(IMU1_CS, HIGH); digitalWrite(IMU2_CS, HIGH);

  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  pinMode(PMIC_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PMIC_INT), isrCalibrate, FALLING);

  // Initialize Haptic Driver
  writeRegisterHaptic(0x01, 0x00); 
  writeRegisterHaptic(0x03, 0x06); // LRA Library
  writeRegisterHaptic(0x16, 0x56); writeRegisterHaptic(0x17, 0xFF);
  writeRegisterHaptic(0x1A, 0xB6); writeRegisterHaptic(0x1B, 0x93);
  writeRegisterHaptic(0x1C, 0x75); writeRegisterHaptic(0x1D, 0x80);
  
  initMagnetometer();
  writeRegisterIMU(IMU1_CS, 0x4E, 0x0F); writeRegisterIMU(IMU2_CS, 0x4E, 0x0F);
  delay(100);

  // --- LOAD SAVED COMPASS DATA ---
  prefs.begin("drum_cal", true); // Read-only mode
  magBiasX = prefs.getFloat("magBiasX", 0.0);
  magBiasY = prefs.getFloat("magBiasY", 0.0);
  magBiasZ = prefs.getFloat("magBiasZ", 0.0);
  magScaleX = prefs.getFloat("magScaleX", 1.0);
  magScaleY = prefs.getFloat("magScaleY", 1.0);
  magScaleZ = prefs.getFloat("magScaleZ", 1.0);
  prefs.end();

  // Signal startup
  triggerLongHapticPulse();
  
  lastMicros = micros();
  stateTimer = millis();
  sysState = STATE_BOOT_WAIT; // Start the 2-second setup wait
  Serial.println("\n🚀 [SYSTEM BOOT] Move to Snare Position. (Wait 2s...)");
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;
  if (dt <= 0 || dt > 0.1) dt = 0.002;

  // --- BUTTON LISTENER FOR FULL CALIBRATION ---
  if (buttonPressed && sysState == STATE_NORMAL) {
    buttonPressed = false;
    sysState = STATE_CAL_WAIT;
    stateTimer = currentMillis;
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.println("\n⏳ [FULL CALIBRATION TRIGGERED] Wait 2 seconds...");
  } else {
    buttonPressed = false; 
  }

  // --- ACQUIRE RAW IMU DATA ---
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  readIMUDataBurst(IMU1_CS, ax1, ay1, az1, gx1, gy1, gz1);
  readIMUDataBurst(IMU2_CS, ax2, ay2, az2, gx2, gy2, gz2);

  // ==========================================
  // FAST BOOTUP SEQUENCE
  // ==========================================
  if (sysState == STATE_BOOT_WAIT) {
    // LED solid while waiting for user to get in position
    digitalWrite(PIN_STATUS_LED, HIGH);
    
    if (currentMillis - stateTimer >= 2000) {
      sysState = STATE_BOOT_QUICK_SNAP;
      stateTimer = currentMillis;
      calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0;
      calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
      calibMagX = 0; calibMagY = 0;
      calibGyroCount = 0; calibMagCount = 0;
      Serial.println("⚡ [QUICK SNAP] Averaging gyros... (Hold still 1.5s)");
    }
    return;
  }

  if (sysState == STATE_BOOT_QUICK_SNAP) {
    // 5 Hz LED Pulse (Rapid flashing)
    if (currentMillis - lastLedToggle >= 100) { 
      lastLedToggle = currentMillis; 
      ledState = !ledState; 
      digitalWrite(PIN_STATUS_LED, ledState); 
    }
    
    calibGyroX1 += gx1; calibGyroY1 += gy1; calibGyroZ1 += gz1;
    calibGyroX2 += gx2; calibGyroY2 += gy2; calibGyroZ2 += gz2;
    calibGyroCount++;

    float mx, my, mz;
    if (readMagnetometer(mx, my, mz)) {
      calibMagX += (mx - magBiasX) * magScaleX;
      calibMagY += (my - magBiasY) * magScaleY;
      calibMagCount++;
    }

    if (currentMillis - stateTimer >= 1500) {
      gyroBiasX1 = (float)calibGyroX1 / calibGyroCount;
      gyroBiasY1 = (float)calibGyroY1 / calibGyroCount;
      gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
      gyroBiasX2 = (float)calibGyroX2 / calibGyroCount;
      gyroBiasY2 = (float)calibGyroY2 / calibGyroCount;
      gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;

      float ax = (ax1 + ax2) / 2.0; 
      float ay = (ay1 + ay2) / 2.0; 
      float az = (az1 + az2) / 2.0;
      pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; 
      targetPitchOffset = pitch; 
      
      float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1);
      float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
      yaw = atan2(avgMagY, avgMagX) * 57.2957; 
      targetYawOffset = yaw;

      triggerHapticPulse(); // One short burst to signify READY
      sysState = STATE_NORMAL;
      Serial.println("🥁 [READY] Boot calibration complete! Start playing.");
    }
    return;
  }

  // ==========================================
  // FULL CALIBRATION OVERRIDE LOGIC
  // ==========================================
  if (sysState == STATE_CAL_WAIT) {
    if (currentMillis - stateTimer >= 2000) {
      sysState = STATE_CAL_FIG8;
      stateTimer = currentMillis;
      magSampleCount = 0;
      triggerLongHapticPulse(); // Signal the start of figure 8
      Serial.println("♾️ [FIGURE-8] Start waving! (15 Seconds)");
    }
    return;
  }
  
  if (sysState == STATE_CAL_FIG8) {
    if (currentMillis - lastMagSampleTime >= 50 && magSampleCount < MAX_MAG_SAMPLES) {
      lastMagSampleTime = currentMillis;
      float mx, my, mz;
      if (readMagnetometer(mx, my, mz)) {
        magBuffer[magSampleCount++] = {mx, my, mz};
      }
    }
    
    if (currentMillis - stateTimer >= 15000) {
      processMagnetometerCalibration(); // Saves to flash internally
      triggerHapticPulse();
      sysState = STATE_CAL_WAIT_SNARE;
      stateTimer = currentMillis;
      digitalWrite(PIN_STATUS_LED, LOW); 
      Serial.println("🛑 [STOP] Move to Snare Position and hold still... (4 Seconds)");
    }
    return;
  }

  if (sysState == STATE_CAL_WAIT_SNARE) {
    if (currentMillis - stateTimer >= 4000) {
      sysState = STATE_CAL_SNARE;
      stateTimer = currentMillis;
      
      calibGyroX1 = 0; calibGyroY1 = 0; calibGyroZ1 = 0;
      calibGyroX2 = 0; calibGyroY2 = 0; calibGyroZ2 = 0;
      calibMagX = 0; calibMagY = 0; 
      calibGyroCount = 0; calibMagCount = 0;
      
      Serial.println("🎯 [SNARE LOCK] Averaging independent IMU biases... (5 Seconds)");
    }
    return;
  }

  if (sysState == STATE_CAL_SNARE) {
    if (currentMillis - lastLedToggle >= 100) { 
      lastLedToggle = currentMillis; 
      ledState = !ledState; 
      digitalWrite(PIN_STATUS_LED, ledState); 
    }
    
    calibGyroX1 += gx1; calibGyroY1 += gy1; calibGyroZ1 += gz1;
    calibGyroX2 += gx2; calibGyroY2 += gy2; calibGyroZ2 += gz2;
    calibGyroCount++;

    float mx, my, mz;
    if (readMagnetometer(mx, my, mz)) {
      calibMagX += (mx - magBiasX) * magScaleX;
      calibMagY += (my - magBiasY) * magScaleY;
      calibMagCount++;
    }

    if (currentMillis - stateTimer >= 5000) {
      gyroBiasX1 = (float)calibGyroX1 / calibGyroCount;
      gyroBiasY1 = (float)calibGyroY1 / calibGyroCount;
      gyroBiasZ1 = (float)calibGyroZ1 / calibGyroCount;
      gyroBiasX2 = (float)calibGyroX2 / calibGyroCount;
      gyroBiasY2 = (float)calibGyroY2 / calibGyroCount;
      gyroBiasZ2 = (float)calibGyroZ2 / calibGyroCount;

      float ax = (ax1 + ax2) / 2.0; 
      float ay = (ay1 + ay2) / 2.0; 
      float az = (az1 + az2) / 2.0;
      pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957; 
      targetPitchOffset = pitch; 
      
      float avgMagX = calibMagX / (calibMagCount > 0 ? calibMagCount : 1);
      float avgMagY = calibMagY / (calibMagCount > 0 ? calibMagCount : 1);
      yaw = atan2(avgMagY, avgMagX) * 57.2957; 
      targetYawOffset = yaw;

      triggerHapticPulse();
      sysState = STATE_NORMAL;
      Serial.println("🥁 [READY] Deep calibration mapped! System Tracking.");
    }
    return;
  }

  // ==========================================
  // NORMAL RUNNING (DYNAMIC FUSION)
  // ==========================================
  
  // 0.5 Hz LED Pulse (Slow heartbeat)
  if (currentMillis - lastLedToggle >= 1000) { 
    lastLedToggle = currentMillis; 
    ledState = !ledState; 
    digitalWrite(PIN_STATUS_LED, ledState); 
  }

  float cleanGy1 = gy1 - gyroBiasY1; 
  float cleanGz1 = gz1 - gyroBiasZ1;
  float cleanGy2 = gy2 - gyroBiasY2;
  float cleanGz2 = gz2 - gyroBiasZ2;

  float gyroPitchRate = ((cleanGy1 + cleanGy2) / 2.0) / 16.4;
  float gyroYawRate   = ((cleanGz1 + cleanGz2) / 2.0) / 16.4;

  float mX, mY, mZ;
  readMagnetometer(mX, mY, mZ);
  float cX = (mX - magBiasX) * magScaleX;
  float cY = (mY - magBiasY) * magScaleY;
  float magHeading = atan2(cY, cX) * 57.2957;
  
  float magDiff = magHeading - yaw;
  while (magDiff > 180.0) magDiff -= 360.0;
  while (magDiff < -180.0) magDiff += 360.0;

  float ax = (ax1 + ax2) / 2.0; 
  float ay = (ay1 + ay2) / 2.0; 
  float az = (az1 + az2) / 2.0;
  
  float accelPitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2957;

  // --- DYNAMIC TRUST REGION ---
  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  float alpha = 0.98;

  // Mask centripetal force during fast swings (1G = ~2048)
  if (accelMag < 1600.0 || accelMag > 2500.0) {
      alpha = 1.0; 
  }

  pitch = alpha * (pitch - gyroPitchRate * dt) + (1.0 - alpha) * accelPitch;
  yaw   = (yaw - gyroYawRate * dt) + (0.02 * magDiff); 

  while (yaw > 180.0) yaw -= 360.0;
  while (yaw < -180.0) yaw += 360.0;

  // --- DRUM ZONE MAPPING ---
  float finalYaw   = yaw - targetYawOffset;
  float finalPitch = pitch - targetPitchOffset;

  while (finalYaw > 180.0) finalYaw -= 360.0;
  while (finalYaw < -180.0) finalYaw += 360.0;

  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint >= 500) {
    lastPrint = currentMillis;

    String drumZone = "SNARE";
    if (finalYaw < -25.0) {
      drumZone = (finalPitch >= 15.0) ? "CRASH" : "HI-HAT";
    } else if (finalYaw >= -25.0 && finalYaw < 0.0) {
      drumZone = (finalPitch >= 15.0) ? "TOM 1" : "SNARE";
    } else if (finalYaw >= 0.0 && finalYaw <= 25.0) {
      drumZone = (finalPitch >= 15.0) ? "TOM 2" : "SNARE";
    } else if (finalYaw > 25.0) {
      drumZone = (finalPitch >= 15.0) ? "RIDE" : "FLOOR TOM";
    }

    Serial.printf("🔍 ZONE: %-10s | [Yaw: %4d°, Pitch: %4d°]\n", drumZone.c_str(), (int)finalYaw, (int)finalPitch);
  }
}