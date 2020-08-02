// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once
#include "mpu6050-regs.hh"

#include "i2c.hh"

#include <array>
#include <type_traits>
#include <cstdint>
#include <cstring>

#define BE16BIT(p) ((p)[0] << 8 | (p)[1])

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
  gyro_data_t read() const;
  void setup_fifo(fifo_e);
  size_t fifo_count() const;
  void empty_fifo();
  void reset_fifo();
  bool is_fifo_enabled() const;

  template<typename T>
  size_t read_fifo_into_buffer(T& readings)
  {
    static_assert(std::is_same<typename T::value_type, gyro_data_t>::value, "wrong container value type");

    std::array<uint8_t, 1024> buffer;
    auto current_fifo_count = fifo_count();
    current_fifo_count -= current_fifo_count % _fifo_datagram_size;
    assert(current_fifo_count % _fifo_datagram_size == 0);
    const size_t count = std::min(
      current_fifo_count / _fifo_datagram_size,
      readings.size()
      );

    _i2c.read_from_device_register_into_buffer(
      _address, MPU6050_FIFO_RW,
      buffer.data(),
      current_fifo_count
    );
    auto p = buffer.data();
    for(size_t i=0; i < count; ++i)
    {
      auto& entry = readings[i];
      std::memset(&entry, 0xff, sizeof(gyro_data_t));

      if(FIFO_EN_ACCEL & _fifo_setup)
      {
        entry.acc[0] = float(BE16BIT(p) - _acc_calibration[0]) / _acc_correction;
        entry.acc[1] = float(BE16BIT(p + 2) - _acc_calibration[1]) / _acc_correction;
        entry.acc[2] = float(BE16BIT(p + 4) - _acc_calibration[2]) / _acc_correction;
        p += 6;
      }
      if(FIFO_EN_TEMP & _fifo_setup)
      {
        p += 2;
      }
      if(FIFO_EN_XG & _fifo_setup)
      {
        entry.gyro[0] = float(BE16BIT(p) - _gyro_calibration[0]) / _gyro_correction;
        p += 2;
      }
      if(FIFO_EN_YG & _fifo_setup)
      {
        entry.gyro[1] = float(BE16BIT(p) - _gyro_calibration[1]) / _gyro_correction;
        p += 2;
      }
      if(FIFO_EN_ZG & _fifo_setup)
      {
        entry.gyro[2] = float(BE16BIT(p) - _gyro_calibration[2]) / _gyro_correction;
        p += 2;
      }
    }
    return count;
  }

private:
  uint8_t read_user_ctrl() const;

  // byte-swapped but otherwise not touched
  std::array<uint8_t, 14> raw_sensor_data() const;

  uint8_t _address;
  I2CHost _i2c;
  // The correction factors to convert
  // from raw readings to d/s or m/s**2
  float _gyro_correction;
  float _acc_correction;
  std::array<int16_t, 3> _gyro_calibration, _acc_calibration;
  size_t _fifo_datagram_size;
  fifo_e _fifo_setup;
};
