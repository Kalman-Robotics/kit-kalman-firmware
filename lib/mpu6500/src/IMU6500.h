#pragma once

#include <Arduino.h>
#include <Wire.h>

#define MPU6500_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

class IMU6500 {
public:
  IMU6500();

  // Initialize MPU6500 on given I2C pins
  bool begin(int sda = 48, int scl = 47, uint32_t freq = 400000);

  // Read accelerometer and gyroscope data
  bool read();

  // Getters for accelerometer data (raw)
  int16_t getAccelX() const { return ax; }
  int16_t getAccelY() const { return ay; }
  int16_t getAccelZ() const { return az; }

  // Getters for gyroscope data (raw)
  int16_t getGyroX() const { return gx; }
  int16_t getGyroY() const { return gy; }
  int16_t getGyroZ() const { return gz; }

  // Get all data at once
  void getData(int16_t& accel_x, int16_t& accel_y, int16_t& accel_z,
               int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z) {
    accel_x = ax;
    accel_y = ay;
    accel_z = az;
    gyro_x = gx;
    gyro_y = gy;
    gyro_z = gz;
  }

private:
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  bool readRegisters(uint8_t regAddr, uint8_t* data, uint8_t len);
};
