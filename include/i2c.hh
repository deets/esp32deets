// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <driver/i2c.h>

void print_error(esp_err_t err);


class I2CHost
{
public:
  I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl);
  ~I2CHost();

  uint8_t read_byte_from_register(uint8_t address, uint8_t register_) const;
  void write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const;
  void read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const;
  template<typename T>
  void read_from_device_register_into_buffer(uint8_t address, uint8_t register_, T& data) const
  {
    const auto size = data.size() * sizeof(typename T::value_type);
    read_from_device_register_into_buffer(address, register_, data.data(), size);
  }

private:
  i2c_port_t _i2c_num;
};
