// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "deets/i2c.hpp"
#include "driver/i2c_master.h"

#include <cstdint>
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

I2CDevice::I2CDevice(I2C& host, uint8_t addr, uint32_t speed)
{
  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = addr,
    .scl_speed_hz = speed
  };
  ESP_ERROR_CHECK(host.add_device(&dev_cfg, &_handle));
}

I2CHost::I2CHost(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, int timeout)
    //, //_timeout(timeout / portTICK_PERIOD_MS)
{
  _config = i2c_master_bus_config_t {
    .i2c_port=i2c_num,
    .sda_io_num=sda,
    .scl_io_num=scl,
    .clk_source=I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt=7,
    .intr_priority=0,
    .trans_queue_depth=0, // Must be 0 for synchronous operations!
    .flags = {
      .enable_internal_pullup=true,
    },
  };
  const auto err = i2c_new_master_bus(&_config, &_handle);
  assert(err == ESP_OK);
}

I2CHost::~I2CHost()
{
  i2c_del_master_bus(_handle);
}

esp_err_t I2CHost::read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const
{
//   esp_err_t err;
//   // send the request
//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
//   i2c_master_write_byte(cmd,  register_, 1);
//   i2c_master_stop(cmd);
//   err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
//   if(err != ESP_OK)
//     goto exit;
//   i2c_cmd_link_delete(cmd);
//   cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
//   i2c_master_read_byte(cmd, &res, i2c_ack_type_t(1));
//   i2c_master_stop(cmd);
//   err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
// exit:
//   i2c_cmd_link_delete(cmd);
//   CHECK_AND_RETURN(err)
  return ESP_OK;
}

esp_err_t I2CHost::write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const
{
  // esp_err_t err;
  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  // i2c_master_write_byte(cmd,  register_, 1);
  // i2c_master_write_byte(cmd,  value, 1);
  // i2c_master_stop(cmd);
  // err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  // i2c_cmd_link_delete(cmd);
  // CHECK_AND_RETURN(err)
  return ESP_OK;
}

esp_err_t I2CHost::write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const
{
  // esp_err_t err;
  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  // i2c_master_write(cmd, const_cast<uint8_t*>(buffer), len, 1);
  // i2c_master_stop(cmd);
  // err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  // i2c_cmd_link_delete(cmd);
  // CHECK_AND_RETURN(err)
  return ESP_OK;
}

esp_err_t I2CHost::write_byte(uint8_t address, uint8_t value) const
{
  // esp_err_t err;
  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
  // i2c_master_write_byte(cmd,  value, 1);
  // i2c_master_stop(cmd);
  // err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  // i2c_cmd_link_delete(cmd);
  // CHECK_AND_RETURN(err)
  return ESP_OK;
}


esp_err_t I2CHost::read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const
{
//   esp_err_t err;
//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
//   i2c_master_write_byte(cmd,  register_, 1);
//   i2c_master_stop(cmd);
//   err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
//   if(err != ESP_OK)
//     goto exit;

//   i2c_cmd_link_delete(cmd);
//   cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
//   i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
//   i2c_master_stop(cmd);
//   err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
// exit:
//   i2c_cmd_link_delete(cmd);
//   CHECK_AND_RETURN(err)
  return ESP_OK;
}

esp_err_t I2CHost::read_from_address_into_buffer(uint8_t address, uint8_t* buffer, size_t length) const
{
  // esp_err_t err;
  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1);
  // i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
  // i2c_master_stop(cmd);
  // err = i2c_master_cmd_begin(_i2c_num, cmd, _timeout);
  // i2c_cmd_link_delete(cmd);
  // CHECK_AND_RETURN(err)
    return ESP_OK;
}

std::vector<uint8_t> I2CHost::scan() const
{
  std::vector<uint8_t> res;
  for(uint8_t address=1; address < 128; ++address)
  {
    const auto result = i2c_master_probe(_handle, address, 50);
    if(result == ESP_OK)
    {
      res.push_back(address);
    }
  }
  return res;
}

void I2CHost::reset()
{
  i2c_master_bus_reset(_handle);

}


std::lock_guard<I2CHost::mutex_type> I2CHost::lock()
{
  return std::lock_guard<mutex_type>(_mutex);
}

esp_err_t I2CHost::add_device(i2c_device_config_t*dev_cfg, i2c_master_dev_handle_t*dev) const
{
  return i2c_master_bus_add_device(_handle, dev_cfg, dev);
}

} // namespace deets::i2c
