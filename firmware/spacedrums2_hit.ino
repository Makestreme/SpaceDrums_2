#include <SPI.h>
#include <Wire.h>

// --- PIN CONFIGURATION ---
const int PIN_SPI_MOSI    = 35;
const int PIN_SPI_MISO    = 37;
const int PIN_SPI_SCK     = 36;
const int PIN_I2C_SDA     = 7;
const int PIN_I2C_SCL     = 8;

const int IMU1_CS         = 5;  // Middle IMU
const int IMU2_CS         = 9;  // Tip IMU
const int PIN_STATUS_LED  = 41;
const int PMIC_KILL       = 11;

const uint8_t DRV2605_ADDR  = 0x5A;

// --- STATE MACHINE ENUMS ---
enum DrumState {
  STATE_IDLE,
  STATE_SWINGING,
  STATE_REFRACTORY
};

DrumState stickState = STATE_IDLE;

// --- HIT DETECTION TUNING ---
// Polarity fixed: Downward swing is POSITIVE Gyro Y.
const int16_t SWING_START_THRESHOLD = 4000;   // Gyro Y must exceed this to start swing
const int16_t HIT_DECEL_THRESHOLD   = 3000;   // Gyro Y must drop by this amount to trigger hit
const unsigned long REFRACTORY_TIME_MS = 100; // Time to ignore bounces

int16_t peak_swing_velocity = 0; 
unsigned long stateTimer = 0;
unsigned long lastMicros = 0;

// --- HARDWARE DRIVERS ---
void writeRegisterIMU(int csPin, uint8_t reg, uint8_t val) {
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x7F); 
  SPI.transfer(val);
  digitalWrite(csPin, HIGH);
}

void readIMUDataBurst(int csPin, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  digitalWrite(csPin, LOW);
  SPI.transfer(0x1F | 0x80); 
  ax = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  ay = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  az = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gx = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gy = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  gz = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
}

void writeRegisterHaptic(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV2605_ADDR); 
  Wire.write(reg); 
  Wire.write(val); 
  Wire.endTransmission();
}

// --- DRV2605L AUTO CALIBRATION ---
void calibrateHaptic() {
  Serial.println("⏳ Starting DRV2605L LRA Auto-Calibration...");
  
  writeRegisterHaptic(0x01, 0x05); // Mode 5: Auto-Calibration
  
  // Hardware properties of LD0832AA (1.8Vrms)
  writeRegisterHaptic(0x16, 0x7A); // Rated Voltage (~1.8Vrms)
  writeRegisterHaptic(0x17, 0x98); // Overdrive Clamp (Increased to ~3.0V Peak for MAX punch)
  
  writeRegisterHaptic(0x1A, 0xB6); // Feedback control: LRA mode
  writeRegisterHaptic(0x1B, 0x93); // Control 1
  writeRegisterHaptic(0x1C, 0x25); // Control 2: Auto-cal time = 1000ms
  writeRegisterHaptic(0x1D, 0x80); // Control 3
  writeRegisterHaptic(0x1E, 0x20); // Control 4: Auto-resonance active
  
  // Start Calibration
  writeRegisterHaptic(0x0C, 0x01); // GO bit
  
  // Wait for calibration to physically run
  delay(1200); 
  
  // Read Status
  Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(0x00); 
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)DRV2605_ADDR, (uint8_t)1);
  uint8_t status = Wire.read();
  
  if ((status & 0x08) == 0) {
    Serial.println("✅ Haptic Calibration SUCCESS!");
  } else {
    Serial.println("❌ Haptic Calibration FAILED. Check motor connection.");
  }
  
  // Return to ROM Mode (Mode 0) and Select Library
  writeRegisterHaptic(0x01, 0x00);
  writeRegisterHaptic(0x03, 0x06); // LRA Library 6
}

void triggerHapticHit(int velocityBucket) {
  uint8_t effect = 0;
  
  // Completely restructured mapping for a drastic dynamic range feel
  switch(velocityBucket) {
    case 8: effect = 1; break; // Strong Click - 100% (Massive Thud)
    case 7: effect = 4; break; // Sharp Click - 100%
    case 6: effect = 2; break; // Strong Click - 60%
    case 5: effect = 5; break; // Sharp Click - 60%
    case 4: effect = 7; break; // Soft Bump - 100%
    case 3: effect = 3; break; // Strong Click - 30%
    case 2: effect = 6; break; // Sharp Click - 30%
    case 1: effect = 8; break; // Soft Bump - 60% (Faint tap)
    default: effect = 4; break;
  }

  writeRegisterHaptic(0x04, effect); // Set Waveform Sequence
  writeRegisterHaptic(0x0C, 1);      // GO!
}

void setup() {
  pinMode(PMIC_KILL, OUTPUT); digitalWrite(PMIC_KILL, HIGH);
  pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, LOW);

  pinMode(IMU1_CS, OUTPUT); digitalWrite(IMU1_CS, HIGH);
  pinMode(IMU2_CS, OUTPUT); digitalWrite(IMU2_CS, HIGH);

  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  // Run Haptic Calibration Routine
  calibrateHaptic();

  // Wake IMUs to Low-Noise Mode
  writeRegisterIMU(IMU1_CS, 0x4E, 0x0F); 
  writeRegisterIMU(IMU2_CS, 0x4E, 0x0F);
  delay(50);

  Serial.println(">>> Polarity-Aware Hit Detection Active <<<");
  lastMicros = micros();
}

void loop() {
  if (micros() - lastMicros < 1000) return; 
  lastMicros = micros();

  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  
  readIMUDataBurst(IMU1_CS, ax1, ay1, az1, gx1, gy1, gz1);
  readIMUDataBurst(IMU2_CS, ax2, ay2, az2, gx2, gy2, gz2);

  // RAW gyro Y (Positive = Downward swing)
  int16_t current_gyro_y = gy2; 
  
  // -----------------------------------------
  // HIT DETECTION STATE MACHINE
  // -----------------------------------------

  if (stickState == STATE_IDLE) {
    // Check for a fast downward movement (Positive spike)
    if (current_gyro_y > SWING_START_THRESHOLD) {
      stickState = STATE_SWINGING;
      peak_swing_velocity = current_gyro_y; 
      digitalWrite(PIN_STATUS_LED, HIGH);
    }
  } 
  
  else if (stickState == STATE_SWINGING) {
    // Keep track of the hardest point of the downward swing
    if (current_gyro_y > peak_swing_velocity) {
      peak_swing_velocity = current_gyro_y;
    }

    // HIT TRIGGER CONDITION:
    // Stick violently stops -> Gyro Y violently drops.
    if (current_gyro_y < (peak_swing_velocity - HIT_DECEL_THRESHOLD)) {
      
      // Map peak [5000 to 25000] into 8 buckets
      int mapped_vel = map(peak_swing_velocity, 5000, 25000, 1, 8);
      
      // Constrain 
      mapped_vel = constrain(mapped_vel, 1, 8);

      // Fire the motor
      triggerHapticHit(mapped_vel);
      
      // Log for Debugging
      Serial.printf("💥 HIT! Vel: %d/8 | Peak Gyro: %d | Stop Gyro: %d\n", mapped_vel, peak_swing_velocity, current_gyro_y);

      // Lock out the system to kill double hits
      stickState = STATE_REFRACTORY;
      stateTimer = millis();
    }
  } 
  
  else if (stickState == STATE_REFRACTORY) {
    digitalWrite(PIN_STATUS_LED, LOW);
    
    // Ignore all bouncing for 100ms
    if (millis() - stateTimer >= REFRACTORY_TIME_MS) {
      stickState = STATE_IDLE;
      peak_swing_velocity = 0;
    }
  }
}