// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "i2c.hh"
#include <esp_log.h>

void print_error(esp_err_t err)
{
  if(err == ESP_OK)
  {
    return;
  }
  char buffer[200];
  esp_err_to_name_r(err, buffer, sizeof(buffer));
  ESP_LOGE("i2c", "error: %s", buffer);
}

I2CHost::I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl)
  : _i2c_num(i2c_num)
{
  i2c_config_t config = {
    .mode=I2C_MODE_MASTER,
    .sda_io_num=sda,
    .sda_pullup_en=GPIO_PULLUP_ENABLE,
    .scl_io_num=scl,
    .scl_pullup_en=GPIO_PULLUP_ENABLE,
  };
  config.master.clk_speed = 400000;

  auto err = i2c_param_config(
    i2c_num,
    &config
    );
  assert(err == ESP_OK);

  err = i2c_driver_install(
    i2c_num,
    I2C_MODE_MASTER,
    0, // rx_buf_len
    0, // tx_buf_len
    0  // alloc flags, currently ignored
    );
  assert(err == ESP_OK);

}

I2CHost::~I2CHost()
{
  i2c_driver_delete(_i2c_num);
}


uint8_t I2CHost::read_byte_from_register(uint8_t address, uint8_t register_) const
{
  uint8_t res;
  esp_err_t err;
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
  i2c_master_read_byte(cmd, &res, i2c_ack_type_t(1));
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  assert(err == ESP_OK);
  i2c_cmd_link_delete(cmd);
  return res;
}

void I2CHost::write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const
{
  esp_err_t err;
  // send the request
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  register_, 1);
  i2c_master_write_byte(cmd,  value, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  assert(err == ESP_OK);
  i2c_cmd_link_delete(cmd);
}

void I2CHost::read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const
{
  esp_err_t err;
  // send the request
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  register_, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  print_error(err);
  assert(err == ESP_OK);
  i2c_cmd_link_delete(cmd);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
  i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  assert(err == ESP_OK);
  i2c_cmd_link_delete(cmd);
}
