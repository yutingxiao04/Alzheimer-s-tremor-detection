/*
  ESP32 + GY-521 (MPU6050) Basic Test
  - Checks WHO_AM_I
  - Prints accelerometer and gyroscope readings
*/

#include <Wire.h>

// I2C pins for ESP32
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// MPU6050 registers
const uint8_t MPU6050_ADDR       = 0x68;  // AD0 low
const uint8_t REG_PWR_MGMT_1     = 0x6B;
const uint8_t REG_WHO_AM_I       = 0x75;
const uint8_t REG_ACCEL_XOUT_H   = 0x3B;

// Sensitivities for +/-2g and +/-250 deg/s (default ranges)
const float ACCEL_SENSITIVITY = 16384.0f; // LSB per g
const float GYRO_SENSITIVITY  = 131.0f;   // LSB per deg/s

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=== MPU6050 Basic Test ===");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // Check WHO_AM_I
  uint8_t whoami = readRegister(MPU6050_ADDR, REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0x68) {
    Serial.println("!! Unexpected WHO_AM_I. MPU6050 may not be connected or may be damaged.");
  } else {
    Serial.println("MPU6050 detected OK (WHO_AM_I = 0x68).");
  }

  // Wake up the MPU6050 (clear SLEEP bit)
  writeRegister(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(100);
  Serial.println("MPU6050 wake-up command sent.");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, tempRaw;
  if (!readAllMPU6050(ax, ay, az, tempRaw, gx, gy, gz)) {
    Serial.println("Failed to read MPU6050 data.");
    delay(500);
    return;
  }

  // Convert to physical units
  float ax_g = ax / ACCEL_SENSITIVITY;
  float ay_g = ay / ACCEL_SENSITIVITY;
  float az_g = az / ACCEL_SENSITIVITY;

  float gx_dps = gx / GYRO_SENSITIVITY;
  float gy_dps = gy / GYRO_SENSITIVITY;
  float gz_dps = gz / GYRO_SENSITIVITY;

  // Temperature (optional)
  // Datasheet: Temp in °C = (tempRaw / 340) + 36.53
  float tempC = (tempRaw / 340.0f) + 36.53f;

  // Print nicely
  Serial.print("ACC (g): ");
  Serial.print("X=");
  Serial.print(ax_g, 3);
  Serial.print("  Y=");
  Serial.print(ay_g, 3);
  Serial.print("  Z=");
  Serial.print(az_g, 3);

  Serial.print("  |  GYRO (deg/s): ");
  Serial.print("X=");
  Serial.print(gx_dps, 2);
  Serial.print("  Y=");
  Serial.print(gy_dps, 2);
  Serial.print("  Z=");
  Serial.print(gz_dps, 2);

  Serial.print("  |  Temp=");
  Serial.print(tempC, 2);
  Serial.println(" C");

  delay(200); // ~5 Hz
}

// ========== Helper functions ==========

uint8_t readRegister(uint8_t devAddr, uint8_t regAddr) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);

  Wire.requestFrom((int)devAddr, 1, true);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Read 14 bytes: accel(6) + temp(2) + gyro(6)
bool readAllMPU6050(int16_t &ax, int16_t &ay, int16_t &az,
                    int16_t &tempRaw,
                    int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom((int)MPU6050_ADDR, 14, true);
  if (Wire.available() < 14) {
    return false;
  }

  ax      = (Wire.read() << 8) | Wire.read();
  ay      = (Wire.read() << 8) | Wire.read();
  az      = (Wire.read() << 8) | Wire.read();
  tempRaw = (Wire.read() << 8) | Wire.read();
  gx      = (Wire.read() << 8) | Wire.read();
  gy      = (Wire.read() << 8) | Wire.read();
  gz      = (Wire.read() << 8) | Wire.read();

  return true;
}

