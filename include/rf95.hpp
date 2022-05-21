// Copyright: 2019, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include <sched.h>
#include <stddef.h>
#include <stdint.h>
#include <array>

class RF95
{
public:
  enum mode_t {
    IDLE,
    TX
  };

  enum class register_t : uint8_t
  {
    FIFO = 0x00,
    PAYLOAD_LENGTH = 0x22,
    OP_MODE = 0x01,
    FREQ_MSB = 0x06,
    FREQ_MID = 0x07,
    FREQ_LSB = 0x08,
    PA_CONFIG = 0x09,
    PA_DAC = 0x4D,
    FIFO_ADDR_PTR = 0xD,
    FIFO_TX_BASE = 0x0E,
    FIFO_RX_BASE = 0x0F,
    MODEM_CONFIG_1 = 0x1D,
    MODEM_CONFIG_2 = 0x1E,
    MODEM_CONFIG_3 = 0x26,
    PREAMBLE_MSB = 0x20,
    PREAMBLE_LSB = 0x21,
  };

  enum class op_reg : uint8_t
  {
    LORA = 1 << 7,
    SLEEP = 0b000,
    STDBY = 0b001,
    FSTX = 0b010,
    TX = 0b011,
    FSRX = 0b100,
    RX = 0b101,
  };

  RF95(spi_host_device_t spi_host, gpio_num_t cs, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso, int speed);
  ~RF95();

  uint8_t reg_read(register_t register_);
  void reg_write(register_t register_, uint8_t value);
  void send(const uint8_t* buffer, size_t len);
private:
  void mode(mode_t mode);
  void setup();
  void modem_config(const std::array<uint8_t, 3>& config);
  void preamble_length(uint16_t);
  void frequency(float);
  void tx_power(int);


  spi_host_device_t _spi_host;
  spi_device_handle_t _spi;
  gpio_num_t _cs;
  gpio_num_t _irq;


};
