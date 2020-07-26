// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <driver/i2c.h>

class I2CHost
{
public:
  I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl);
  ~I2CHost();

  uint8_t read_byte_from_register(uint8_t address, uint8_t register_) const;
  void write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const;
  template<typename T>
  void read_from_device_register_into_buffer(uint8_t address, uint8_t register_, T& data) const
  {
    esp_err_t err;
    const auto size = data.size() * sizeof(typename T::value_type);
    // send the request
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd,  register_, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
    assert(err == ESP_OK);
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data.data(), size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
    assert(err == ESP_OK);
    i2c_cmd_link_delete(cmd);
  }

private:
  i2c_port_t _i2c_num;
};
