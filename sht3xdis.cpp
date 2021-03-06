// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#include "sht3xdis.hpp"

#include "i2c.hh"

#include <array>

namespace sht3xdis {

namespace {

const auto MEASUREMENT_TIME = 20; // ms, according to datasheet, 15. I add a bit extra.

// the underscore is needed due to a name collision
const auto STATUS_ = std::array<uint8_t, 2>{0xF3, 0x2D};
const auto MEASUREMENT = std::array<uint8_t, 2>{0x24, 0x00};
const auto RESET = std::array<uint8_t, 2>{0x30, 0xA2};
const auto CLEAR = std::array<uint8_t, 2>{0x30, 0x41};

} // namespace


bool SHT3XDIS::valid_address(uint8_t address)
{
  return address == 0x44 || address == 0x45;
}

SHT3XDIS::SHT3XDIS(I2C &bus, uint8_t address)
  : _bus(bus)
  , _address(address)
{
}

RawValues SHT3XDIS::raw_values()
{
  _bus.write_buffer_to_address(_address, MEASUREMENT.data(), MEASUREMENT.size());
  vTaskDelay(MEASUREMENT_TIME / portTICK_PERIOD_MS);

  std::array<uint8_t, 6> buffer;
  _bus.read_from_address_into_buffer(_address, buffer.data(), buffer.size());
  const auto humidity = uint16_t(buffer[0] << 8 | buffer[1]);
  const auto temp = uint16_t(buffer[3] << 8 | buffer[4]);
  return { humidity, temp };
}

Values SHT3XDIS::values()
{
  return { 0.0, 0.0 };
}


void SHT3XDIS::reset()
{
  _bus.write_buffer_to_address(_address, RESET.data(), RESET.size());
}

void SHT3XDIS::clear()
{
  _bus.write_buffer_to_address(_address, CLEAR.data(), CLEAR.size());
}

uint16_t SHT3XDIS::status()
{
  _bus.write_buffer_to_address(_address, STATUS_.data(), STATUS_.size());
  std::array<uint8_t, 3> buffer;
  _bus.read_from_address_into_buffer(_address, buffer.data(), buffer.size());
  return buffer[0] << 8 | buffer[1];
}

}
