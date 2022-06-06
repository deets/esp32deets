// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include "i2c.hh"
#include <array>

class TCA9548A
{
  class WrappedBus : public I2C
  {
    esp_err_t write_byte(uint8_t address, uint8_t value) const override;
    esp_err_t read_byte_from_register(uint8_t address, uint8_t register_, uint8_t& res) const override;
    esp_err_t write_byte_to_register(uint8_t address, uint8_t register_, uint8_t value) const override;
    esp_err_t write_buffer_to_address(uint8_t address, const uint8_t* buffer, size_t len) const override;
    esp_err_t read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const override;
    esp_err_t read_from_address_into_buffer(uint8_t address, uint8_t *buffer, size_t length) const override;
    std::vector<uint8_t> scan() const override;

  private:
    WrappedBus(uint8_t busno, TCA9548A& mux, I2C& bus);

    uint8_t _busno;
    TCA9548A& _mux;
    I2C& _bus;

    friend TCA9548A;
  };


public:
  TCA9548A(I2C& bus, uint8_t address=0x70);

  I2C& bus(uint8_t channel);

private:
  void select(uint8_t busno);

  std::array<WrappedBus, 8> _busses;
  uint8_t _address;
  uint8_t _selected_bus;
  I2C& _bus;
};
