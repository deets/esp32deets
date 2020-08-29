// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once
#include "mpu6050-regs.hh"

#include "i2c.hh"

#include <array>
#include <type_traits>
#include <cstdint>
#include <cstring>

class MPU6050
{

public:
  struct gyro_data_t {
    float acc[3];
    float gyro[3];
  };

  typedef enum {
    GYRO_250_FS=0,
    GYRO_500_FS=1,
    GYRO_1000_FS=2,
    GYRO_2000_FS=3
  } gyro_fs_e;

  typedef enum {
    ACC_2_FS=0,
    ACC_4_FS=1,
    ACC_8_FS=2,
    ACC_16_FS=3
  } acc_fs_e;

  typedef enum {
    FIFO_EN_NONE=0x00,
    FIFO_EN_TEMP=0x80,
    FIFO_EN_XG=0x40,
    FIFO_EN_YG=0x20,
    FIFO_EN_ZG=0x10,
    FIFO_EN_ALLG=0x70,
    FIFO_EN_ACCEL=0x08,
    FIFO_EN_SLV2=0x04,
    FIFO_EN_SLV1=0x02,
    FIFO_EN_SLV0=0x01
  } fifo_e;

  MPU6050(uint8_t address, I2CHost& i2c, gyro_fs_e gyro_scale, acc_fs_e acc_scale);

  void set_gyro_scale(gyro_fs_e);
  void set_acc_scale(acc_fs_e);
  bool calibrate(size_t iterations);
  void calibrate_fifo_based();
  gyro_data_t read() const;
  void setup_fifo(fifo_e);
  size_t fifo_count() const;
  void empty_fifo();
  void reset_fifo();
  bool fifo_enabled() const;
  bool fifo_overflown() const;
  size_t samplerate() const;
  void samplerate(uint8_t value);

  template<typename T>
  void consume_fifo(T callback);

private:
   uint8_t* populate_entry(uint8_t* p, gyro_data_t& entry);

  uint8_t read_user_ctrl() const;

  // byte-swapped but otherwise not touched
  std::array<uint8_t, 14> raw_sensor_data() const;

  uint8_t _address;
  I2CHost _i2c;
  // The scale factors to convert
  // from raw readings to d/s or m/s**2
  float _gyro_scale;
  float _acc_scale;
  std::array<int16_t, 3> _gyro_calibration, _acc_calibration;
  size_t _fifo_datagram_size;
  fifo_e _fifo_setup;
};

#include "mpu6050.ipp"
