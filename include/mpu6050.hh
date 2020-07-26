// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include "i2c.hh"

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

  gyro_data_t read_raw() const;

private:
  uint8_t _address;
  I2CHost _i2c;
  float _gyro_correction;
  float _acc_correction;
};
