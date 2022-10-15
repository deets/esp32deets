// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include <sdkconfig.h>
#ifndef CONFIG_DEETS_USE_I2C
#error "CONFIG_DEETS_USE_I2C not set!"
#endif
#pragma once

#include <driver/i2c.h>

#include <vector>
#include <mutex>


void print_error(esp_err_t err);


class I2C {
public:
  virtual esp_err_t write_byte(uint8_t address, uint8_t value) const = 0;
  virtual esp_err_t read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const = 0;
  virtual esp_err_t write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const = 0;
  virtual esp_err_t write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const = 0;
  virtual esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const = 0;
  virtual esp_err_t read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const = 0;
  virtual std::lock_guard<std::mutex> lock() = 0;

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
  I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, int timeout=100);
  ~I2CHost();

  esp_err_t set_speed(int speed);
  esp_err_t write_byte(uint8_t address, uint8_t value) const override;
  esp_err_t read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const override;
  esp_err_t write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const override;
  esp_err_t write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const override;
  esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const override;
  esp_err_t read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const override;
  std::lock_guard<std::mutex> lock() override;

  std::vector<uint8_t> scan() const override;
  void reset();

private:
  i2c_config_t _config;
  i2c_port_t _i2c_num;
  intr_handle_t _i2c_intr_handle;
  TickType_t _timeout;
  std::mutex _mutex;
};
