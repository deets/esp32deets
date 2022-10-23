// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "deets/i2c.hpp"
#include <esp_log.h>
#include <vector>

#define TAG "i2c"

#define CHECK_AND_RETURN(err) \
  if(err == ESP_ERR_TIMEOUT) \
  { \
    ESP_LOGD(TAG, "Bus timed out, resetting");  \
  } \
  return err;


namespace deets::i2c {

I2CHost::I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, int timeout)
  : _i2c_num(i2c_num)
  , _timeout(timeout / portTICK_RATE_MS)
{
  _config = i2c_config_t {
    .mode=I2C_MODE_MASTER,
    .sda_io_num=sda,
    .scl_io_num=scl,
    .sda_pullup_en=GPIO_PULLUP_ENABLE,
    .scl_pullup_en=GPIO_PULLUP_ENABLE,
    .clk_flags=0,
  };
  // implicitly configures the
  // bus
  auto err = set_speed(400000);

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

esp_err_t I2CHost::set_speed(int speed)
{
  _config.master.clk_speed = speed;
  return i2c_param_config(
    _i2c_num,
    &_config
    );
}

esp_err_t I2CHost::read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const
{
  esp_err_t err;
  // send the request
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  register_, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  if(err != ESP_OK)
    goto exit;
  i2c_cmd_link_delete(cmd);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
  i2c_master_read_byte(cmd, &res, i2c_ack_type_t(1));
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
exit:
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}

esp_err_t I2CHost::write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  register_, 1);
  i2c_master_write_byte(cmd,  value, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}

esp_err_t I2CHost::write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write(cmd, const_cast<uint8_t*>(buffer), len, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}

esp_err_t I2CHost::write_byte(uint8_t address, uint8_t value) const
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  value, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}


esp_err_t I2CHost::read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd,  register_, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  if(err != ESP_OK)
    goto exit;

  i2c_cmd_link_delete(cmd);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
  i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
exit:
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}

esp_err_t I2CHost::read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
  i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  i2c_cmd_link_delete(cmd);
  CHECK_AND_RETURN(err)
}

std::vector<uint8_t> I2CHost::scan() const
{
  std::vector<uint8_t> res;
  for(uint8_t address=1; address < 128; ++address)
  {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    const auto result = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
    if(result == ESP_OK)
    {
      res.push_back(address);
    }
  }
  return res;
}

void I2CHost::reset()
{
  i2c_reset_rx_fifo(_i2c_num);
  i2c_reset_tx_fifo(_i2c_num);
}


std::lock_guard<I2CHost::mutex_type> I2CHost::lock()
{
  return std::lock_guard<mutex_type>(_mutex);
}

} // namespace deets::i2c
