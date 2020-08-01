// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include "i2c.hh"

#include <array>
#include <cstdint>

#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69

class MPU6050
{

public:
  typedef enum {
    GYRO_250_FS=0,
    GYRO_500_FS=1,
    GYRO_1000_FS=2,
    GYRO_2000_FS=3
  } gyro_fs_t;

  typedef enum {
    ACC_2_FS=0,
    ACC_4_FS=1,
    ACC_8_FS=2,
    ACC_16_FS=3
  } acc_fs_t;

  struct gyro_data_t {
    float acc[3];
    float gyro[3];
  };

  MPU6050(uint8_t address, I2CHost& i2c, gyro_fs_t gyro_scale, acc_fs_t acc_scale);

  void set_gyro_scale(gyro_fs_t);
  void set_acc_scale(acc_fs_t);
  bool calibrate(size_t iterations);

  gyro_data_t read_raw() const;

private:
  // byte-swapped but otherwise not touched
  std::array<uint8_t, 14> raw_sensor_data() const;

  uint8_t _address;
  I2CHost _i2c;
  // The correction factors to convert
  // from raw readings to d/s or m/s**2
  float _gyro_correction;
  float _acc_correction;
  std::array<int16_t, 3> _gyro_calibration, _acc_calibration;
};
