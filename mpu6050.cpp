// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "mpu6050.hh"
#include <esp_log.h>

#include <cstring>
#include <algorithm>

#define MPU6050_RA_WHO_AM_I 0x75

#define MPU6050_RA_PWR_MGMT_1  0x6B
#define MPU6050_RA_PWR_MGMT_2  0x6C
#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

#define MPU6050_DEFAULT_SAMPLE_RATE 0x20
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

namespace {

template<typename>
struct array_size;
template<typename T, size_t N>
struct array_size<std::array<T,N> > {
    static size_t const size = N;
};

const auto GYRO_CALIBRATION_VARIANCE = 20;

template<typename T>
int16_t compute_average(const T& buffer)
{
  int32_t accu = 0;
  accu = std::accumulate(buffer.cbegin(), buffer.cend(), accu);
  accu /=  buffer.size();
  return (int16_t)accu;
}

template<typename T>
int16_t compute_variance(const T& buffer)
{
  // I don't really remember why this works
  // as variance computation, but I probably derived
  // it somehow during the new joy project.
  const auto size = buffer.size();
  int32_t accu = 0;
  for(size_t i=0; i < size - 1; ++i)
  {
    accu += abs(buffer[i] - buffer[(i + 1)]);
  }
  accu /= size - 1;
  return (int16_t)accu;
}

} // end ns anonymous


MPU6050::MPU6050(uint8_t address, I2CHost& i2c, gyro_fs_t gyro_scale, acc_fs_t acc_scale)
  : _address(address)
  , _i2c(i2c)
{
  const auto who_am_i = _i2c.read_byte_from_register(_address, MPU6050_RA_WHO_AM_I);

  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_CLOCK_PLL_XGYRO
    );

  // enable all sensors
  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_PWR_MGMT_2,
    0
    );

  // set sampling rate
  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_SMPLRT_DIV,
    MPU6050_DEFAULT_SAMPLE_RATE
    );

  // set digital low pass filtering
  // 0 means none of that
  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_CONFIG,
    0
    );

  set_gyro_scale(gyro_scale);
  set_acc_scale(acc_scale);
}


bool MPU6050::calibrate(size_t iterations)
{
  using buffer_t = std::array<int16_t, 100>;
  std::array<buffer_t, 7> all_sensor_data;
  for(size_t i=0; i < iterations; ++i)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
    auto raw = raw_sensor_data();
    auto p = reinterpret_cast<uint16_t*>(raw.data());
    for(auto k=0; k < 7; ++k)
    {
      all_sensor_data[k][i % array_size<buffer_t>::size] = *p++;
    }

    // we got one full buffer,
    // try and see if we have been resting
    if(i % array_size<buffer_t>::size == array_size<buffer_t>::size - 1)
    {
      // when all three are below our threshold, we consider this to be good.
      if(
        compute_variance(all_sensor_data[4])< GYRO_CALIBRATION_VARIANCE && \
        compute_variance(all_sensor_data[5])< GYRO_CALIBRATION_VARIANCE && \
        compute_variance(all_sensor_data[6])< GYRO_CALIBRATION_VARIANCE)
        {
          // I piggy-back on the gyros being stable, as I presume there is also no acceleration going on then
          for(size_t i=0; i < 3; ++i)
          {
            _gyro_calibration[i] = compute_average(all_sensor_data[4 + i]);
            _acc_calibration[i] = compute_average(all_sensor_data[i]);
          }
          return true;
        }
    }
  }
  return false;
}


void MPU6050::set_gyro_scale(gyro_fs_t scale)
{
  auto gyro_range = _i2c.read_byte_from_register(
    _address,
    MPU6050_RA_GYRO_CONFIG
    );

  gyro_range |= ~(3 << 3);
  gyro_range |= scale << 3;
  _i2c.write_byte_to_register(
    _address,
    MPU6050_RA_GYRO_CONFIG,
    gyro_range
    );

  switch(scale)
  {
  case GYRO_250_FS:
    _gyro_correction = 32768 / 250.0;
    break;
  case GYRO_500_FS:
    _gyro_correction = 32768 / 500.0;
    break;
  case GYRO_1000_FS:
    _gyro_correction = 32768 / 1000.0;
    break;
  case GYRO_2000_FS:
    _gyro_correction = 32768 / 2000.0;
    break;
  }
}

void MPU6050::set_acc_scale(acc_fs_t acc_scale)
{
   uint8_t acc_range = _i2c.read_byte_from_register(
     _address,
     MPU6050_RA_ACCEL_CONFIG
     );
  acc_range |= ~(3 << 3);
  acc_range |= acc_scale << 3;
  _i2c.write_byte_to_register(
    _address,
    MPU6050_RA_ACCEL_CONFIG,
    acc_range
    );

  switch(acc_scale)
  {
  case ACC_2_FS:
    _acc_correction = 32768.0 / 2;
    break;
  case ACC_4_FS:
    _acc_correction = 32768.0 / 4;
    break;
  case ACC_8_FS:
    _acc_correction = 32768.0 / 8;
    break;
  case ACC_16_FS:
    _acc_correction = 32768.0 / 16;
    break;
  }
}

std::array<uint8_t, 14> MPU6050::raw_sensor_data() const
{
  std::array<uint8_t, 14> raw; // acc + temp + gyro data
  _i2c.read_from_device_register_into_buffer(
    _address,
    MPU6050_RA_ACCEL_XOUT_H,
    raw
    );
  // swap endianess
  for(size_t i=0; i < 7; ++i)
  {
    uint8_t h = raw[i*2];
    raw[i*2]= raw[i*2 + 1];
    raw[i*2 + 1] = h;
  }
  return raw;
}

MPU6050::gyro_data_t MPU6050::read_raw() const
{
  gyro_data_t res;
  const auto raw = raw_sensor_data();
  const auto word_access = (const int16_t*)raw.data();
  for(size_t i=0; i < 3; ++i)
  {
    res.gyro[i] = (float)(word_access[3 + 1 + i] - _gyro_calibration[i]) / _gyro_correction;
    res.acc[i] = (float)(word_access[i] - _acc_calibration[i]) / _acc_correction;
  }
  return res;
}


/*

#include "madgwick-ahrs.h"

#include <string.h>

#define GYRO_CALIBRATION_BUFFER_SIZE 10



typedef struct {
  mpu6050_gyro_fs_t gyro_fs;
  mpu6050_acc_fs_t acc_fs;
  uint8_t input_buffer[14]; // acc + temp + gyro data
  float acc_correction;
  float gyro_correction;
} mpu6050_task_data_t;



void newjoy_task_mpu6050(nj_task_def_t* task, uint8_t *buffer)
{
  mpu6050_task_data_t* task_data = (mpu6050_task_data_t*)task->task_data;
  read_from_device_register_into_buffer(
_i2c,
    _address,
    MPU6050_RA_ACCEL_XOUT_H,
    task_data->input_buffer,
    sizeof(task_data->input_buffer)
    );
  // swap endianess
  for(size_t i=0; i < 7; ++i)
  {
    uint8_t h = task_data->input_buffer[i*2];
    task_data->input_buffer[i*2]= task_data->input_buffer[i*2 + 1];
    task_data->input_buffer[i*2 + 1] = h;
  }
  float gyro_data[3];
  float acc_data[3];

  int16_t* word_access = (int16_t*)task_data->input_buffer;

  switch(task_data->gyro_calbration_mode)
  {
  case GYRO_UNCALIBRATED:
  case GYRO_CALIBRATED:
    for(size_t i=0; i < 3; ++i)
    {
      gyro_data[i] = ((float)(word_access[3 + 1 + i] - task_data->gyro_calibration[i])) / task_data->gyro_correction;
      acc_data[i] = ((float)(word_access[i] - task_data->acc_calibration[i])) / task_data->acc_correction;
    }
    madgwick_ahrs_update_imu(
      &task_data->filter_data,
      gyro_data[0], gyro_data[1], gyro_data[2],
      acc_data[0], acc_data[1], acc_data[2]
      );
    float* quaternion_data = (float*)buffer;
    *quaternion_data++ = task_data->filter_data.q0;
    *quaternion_data++ = task_data->filter_data.q1;
    *quaternion_data++ = task_data->filter_data.q2;
    *quaternion_data++ = task_data->filter_data.q3;
    *quaternion_data++ = acc_data[0];
    *quaternion_data++ = acc_data[1];
    *quaternion_data++ = acc_data[2];
    break;
  }
}


int newjoy_task_setup_mpu6050(nj_task_def_t* task, int period)
{
  int res;
  uint8_t identity;
  // first, identify ourselves. We should
  // get our own address back.
  res = read_byte_from_device_register(
_i2c,
    _address,
    MPU6050_RA_WHO_AM_I,
    &identity
    );
  if(res)
  {
    return res;
  }
  if(identity != MPU6050_DEFAULT_ADDRESS && identity != 0x72)
  {
    return -1;
  }

  mpu6050_task_data_t* task_data = task->task_data = malloc(sizeof(mpu6050_task_data_t));
  // disable sleep mode and select clock source
  i2c.write_byte_to_register(
_i2c,
    _address,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_CLOCK_PLL_XGYRO
    );

  // enable all sensors
  i2c.write_byte_to_register(
_i2c,
    _address,
    MPU6050_RA_PWR_MGMT_2,
    0
    );

  // set sampling rate
  i2c.write_byte_to_register(
_i2c,
    _address,
    MPU6050_RA_SMPLRT_DIV,
    MPU6050_DEFAULT_SAMPLE_RATE
    );

  // enable dlpf
  i2c.write_byte_to_register(
_i2c,
    _address,
    MPU6050_RA_CONFIG,
    1
    );


  task_data->gyro_calibration_buffer_fill = 0;
  task_data->gyro_calbration_mode = GYRO_UNCALIBRATED;
  madgwick_ahrs_init(
    &task_data->filter_data,
    1000 / period // period is in ms
    );
  return 0;
}
*/
