// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "mpu6050.hh"
#include <esp_log.h>

#include <cstring>
#include <algorithm>

#define BE16BIT(p) ((p)[0] << 8 | (p)[1])

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


MPU6050::MPU6050(uint8_t address, I2CHost& i2c, gyro_fs_e gyro_scale, acc_fs_e acc_scale)
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

  // set digital low pass filtering
  // 0 means none of that
  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_CONFIG,
    0
    );

  // Set sampling rate. Because we
  // don't have a digital low pass filtering,
  // it would be 8KHz - and we are only
  // interested in 1KHz for now
  i2c.write_byte_to_register(
    _address,
    MPU6050_RA_SMPLRT_DIV,
    7 // 1 + X is the actual divider
    );

  set_gyro_scale(gyro_scale);
  set_acc_scale(acc_scale);
  setup_fifo(FIFO_EN_NONE);
  _gyro_calibration.fill(0);
  _acc_calibration.fill(0);
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

void MPU6050::calibrate_fifo_based()
{
  std::array<int32_t, 6> accu;
  accu.fill(0);
  const auto sample_number = samplerate();
  if(_fifo_setup == 0)
  {
    ESP_LOGE("mpu", "No FIFO configured, can't calibrate using FIFO");
    return;
  }

  auto count = 0;
  while(count < sample_number)
  {
    consume_fifo(
      [this, &accu, &count](const MPU6050::gyro_data_t& entry)
      {
        if(_fifo_setup & FIFO_EN_ACCEL)
        {
          accu[0] += int32_t(entry.acc[0] * _acc_scale);
          accu[1] += int32_t(entry.acc[1] * _acc_scale);
          accu[2] += int32_t(entry.acc[2] * _acc_scale);
        }
        if(_fifo_setup & FIFO_EN_XG)
        {
          accu[3] += int32_t(entry.gyro[0] * _gyro_scale);
        }
        if(_fifo_setup & FIFO_EN_YG)
        {
          accu[4] += int32_t(entry.gyro[1] * _gyro_scale);
        }
        if(_fifo_setup & FIFO_EN_ZG)
        {
          accu[5] += int32_t(entry.gyro[2] * _gyro_scale);
        }
        // This must be counted in here, because
        // we are counting *actual* entries consumed
        ++count;
      }
      );
  }
  if(_fifo_setup & FIFO_EN_ACCEL)
  {
    _acc_calibration[0] = accu[0] / count;
    _acc_calibration[1] = accu[1] / count;
    _acc_calibration[2] = accu[2] / count;
  }
  if(_fifo_setup & FIFO_EN_XG)
  {
    _gyro_calibration[0] = accu[3] / count;
  }
  if(_fifo_setup & FIFO_EN_YG)
  {
    _gyro_calibration[1] = accu[4] / count;
  }
  if(_fifo_setup & FIFO_EN_ZG)
  {
    ESP_LOGI("mpu", "accu gz: %i, count: %i", accu[5], count);
    _gyro_calibration[2] = accu[5] / count;
  }
}


void MPU6050::set_gyro_scale(gyro_fs_e scale)
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
    _gyro_scale = 32768 / 250.0;
    break;
  case GYRO_500_FS:
    _gyro_scale = 32768 / 500.0;
    break;
  case GYRO_1000_FS:
    _gyro_scale = 32768 / 1000.0;
    break;
  case GYRO_2000_FS:
    _gyro_scale = 32768 / 2000.0;
    break;
  }
}

void MPU6050::set_acc_scale(acc_fs_e acc_scale)
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
    _acc_scale = 32768.0 / 2;
    break;
  case ACC_4_FS:
    _acc_scale = 32768.0 / 4;
    break;
  case ACC_8_FS:
    _acc_scale = 32768.0 / 8;
    break;
  case ACC_16_FS:
    _acc_scale = 32768.0 / 16;
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

MPU6050::gyro_data_t MPU6050::read() const
{
  gyro_data_t res;
  const auto raw = raw_sensor_data();
  const auto word_access = (const int16_t*)raw.data();
  for(size_t i=0; i < 3; ++i)
  {
    res.gyro[i] = (float)(word_access[3 + 1 + i] - _gyro_calibration[i]) / _gyro_scale;
    res.acc[i] = (float)(word_access[i] - _acc_calibration[i]) / _acc_scale;
  }
  return res;
}

void MPU6050::setup_fifo(fifo_e fifo_setup)
{
  _i2c.write_byte_to_register(
    _address,
    MPU6050_FIFO_EN,
    fifo_setup
    );
  auto user_ctrl = read_user_ctrl();
  if(fifo_setup == FIFO_EN_NONE)
  {
    user_ctrl &= ~FIFO_EN;
  }
  else
  {
    user_ctrl |= FIFO_EN | FIFO_RESET;
  }
  _i2c.write_byte_to_register(_address, MPU6050_RA_USER_CTRL, user_ctrl);
  _fifo_datagram_size = 0;
  _fifo_datagram_size += 2 * (fifo_setup & FIFO_EN_TEMP ? 1 : 0);
  _fifo_datagram_size += 2 * (fifo_setup & FIFO_EN_XG ? 1 : 0);
  _fifo_datagram_size += 2 * (fifo_setup & FIFO_EN_YG ? 1 : 0);
  _fifo_datagram_size += 2 * (fifo_setup & FIFO_EN_ZG ? 1 : 0);
  _fifo_datagram_size += 6 * (fifo_setup & FIFO_EN_ACCEL ? 1 : 0);
  assert(!(FIFO_EN_SLV0 & fifo_setup));
  assert(!(FIFO_EN_SLV1 & fifo_setup));
  assert(!(FIFO_EN_SLV2 & fifo_setup));
  _fifo_setup = fifo_setup;
  ESP_LOGI("mpu", "fifo_datagram_size: %i", _fifo_datagram_size);
  empty_fifo();
}


size_t MPU6050::fifo_count() const
{
  std::array<uint8_t, 2> raw; // acc + temp + gyro data
  _i2c.read_from_device_register_into_buffer(
    _address,
    MPU6050_FIFO_COUNT_H,
    raw
    );
  return raw[0] << 8 | raw[1];
}

size_t MPU6050::samplerate() const
{
  const auto divider = _i2c.read_byte_from_register(
    _address,
    MPU6050_RA_SMPLRT_DIV
    ) + 1;
  const auto digital_filter = _i2c.read_byte_from_register(
    _address,
    MPU6050_RA_CONFIG
    ) & 0x7;

  const auto output_rate = (digital_filter == 0 || digital_filter == 7) ? 8000 : 1000;
  return output_rate / divider;
}

uint8_t MPU6050::read_user_ctrl() const
{
  return _i2c.read_byte_from_register(_address, MPU6050_RA_USER_CTRL);
}


void MPU6050::reset_fifo()
{
  const auto user_ctrl = read_user_ctrl();
  _i2c.write_byte_to_register(_address, MPU6050_RA_USER_CTRL, user_ctrl | FIFO_RESET);
}

bool MPU6050::is_fifo_enabled() const
{
  return read_user_ctrl() | FIFO_EN;
}

void MPU6050::empty_fifo()
{
  std::array<uint8_t, 1024> buffer;
  const auto count = fifo_count();
  if(count)
  {
    _i2c.read_from_device_register_into_buffer(
      _address, MPU6050_FIFO_RW,
      buffer.data(),
      count
      );
    reset_fifo();
  }
}

uint8_t* MPU6050::populate_entry(uint8_t* p, gyro_data_t& entry)
{
  if(FIFO_EN_ACCEL & _fifo_setup)
  {
    entry.acc[0] = float(BE16BIT(p) - _acc_calibration[0]) / _acc_scale;
    entry.acc[1] = float(BE16BIT(p + 2) - _acc_calibration[1]) / _acc_scale;
    entry.acc[2] = float(BE16BIT(p + 4) - _acc_calibration[2]) / _acc_scale;
    p += 6;
  }
  if(FIFO_EN_TEMP & _fifo_setup)
  {
    p += 2;
  }
  if(FIFO_EN_XG & _fifo_setup)
  {
    entry.gyro[0] = float(BE16BIT(p) - _gyro_calibration[0]) / _gyro_scale;
    p += 2;
  }
  if(FIFO_EN_YG & _fifo_setup)
  {
    entry.gyro[1] = float(BE16BIT(p) - _gyro_calibration[1]) / _gyro_scale;
    p += 2;
  }
  if(FIFO_EN_ZG & _fifo_setup)
  {
    entry.gyro[2] = float(BE16BIT(p) - _gyro_calibration[2]) / _gyro_scale;
    p += 2;
  }
  return p;
}
