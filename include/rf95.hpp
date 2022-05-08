// Copyright: 2019, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include <stddef.h>
#include <stdint.h>

class RF95
{
public:
  RF95(spi_host_device_t spi_host, gpio_num_t cs, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso, int speed);
  ~RF95();

  uint8_t reg_read(uint8_t register_);
  void reg_write(uint8_t register_, uint8_t value);

private:
  //uint8_t reg_write(uint8_t reg, uint8_t value);

  spi_host_device_t _spi_host;
  spi_device_handle_t _spi;
  gpio_num_t _cs;
  gpio_num_t _irq;


};
