#include "IMU6500.h"

IMU6500::IMU6500() : ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {
}

bool IMU6500::begin(int sda, int scl, uint32_t freq) {
  // Initialize I2C
  Wire.begin(sda, scl, freq);

  // Wake up MPU6500
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Set to zero (wakes up the MPU-6500)
  Wire.endTransmission(true);

  delay(100);  // Wait for startup

  return true;
}

bool IMU6500::read() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)MPU6500_ADDR, (size_t)6, (bool)true) != 6) {
    return false;
  }

  // ✅ IMPORTANTE: Cast a int16_t para valores con signo
  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  // Read gyroscope data
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)MPU6500_ADDR, (size_t)6, (bool)true) != 6) {
    return false;
  }

  // ✅ IMPORTANTE: Cast a int16_t para valores con signo
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}
bool IMU6500::readRegisters(uint8_t regAddr, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(regAddr);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)MPU6500_ADDR, (size_t)len, (bool)true) != len) {
    return false;
  }

  for (int i = 0; i < len; i++) {
    data[i] = Wire.read();
  }

  return true;
}
