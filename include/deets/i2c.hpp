// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "driver/i2c_types.h"
#include "esp_err.h"
#include <cstdint>
#include <sdkconfig.h>
#ifndef CONFIG_DEETS_USE_I2C
#error "CONFIG_DEETS_USE_I2C not set!"
#endif
#pragma once

#include <driver/i2c_master.h>

#include <vector>
#include <mutex>

namespace deets::i2c {
class I2C;

class I2CDevice {
public:
  I2CDevice(I2C& host, uint8_t addr, uint32_t speed=400000);
private:

  i2c_master_dev_handle_t _handle;
};

class I2C {
public:
  using mutex_type = std::recursive_mutex;

  virtual esp_err_t write_byte(uint8_t address, uint8_t value) const = 0;
  virtual esp_err_t read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const = 0;
  virtual esp_err_t write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const = 0;
  virtual esp_err_t write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const = 0;
  virtual esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const = 0;
  virtual esp_err_t read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const = 0;
  virtual esp_err_t add_device(i2c_device_config_t*, i2c_master_dev_handle_t*) const = 0;

  virtual std::lock_guard<mutex_type> lock() = 0;

  template<typename T>
  esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, T& data) const
  {
    const auto size = data.size() * sizeof(typename T::value_type);
    return read_from_device_register_into_buffer(address, register_, data.data(), size);
  }

  virtual std::vector<uint8_t> scan() const = 0;
};

class I2CHost : public I2C
{
public:
  using mutex_type = I2C::mutex_type;

  I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, int timeout=100);
  ~I2CHost();

  esp_err_t set_speed(int speed);
  esp_err_t write_byte(uint8_t address, uint8_t value) const override;
  esp_err_t read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const override;
  esp_err_t write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const override;
  esp_err_t write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const override;
  esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const override;
  esp_err_t read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const override;
  esp_err_t add_device(i2c_device_config_t*, i2c_master_dev_handle_t*) const override;
  std::lock_guard<mutex_type> lock() override;

  std::vector<uint8_t> scan() const override;
  void reset();

  i2c_master_bus_handle_t handle() const {
    return _handle;
  }
private:
  friend class I2CDevice;

  i2c_master_bus_config_t _config;
  i2c_master_bus_handle_t _handle;
//  TickType_t _timeout;
  mutex_type _mutex;
};

} // namespace deets::i2c
