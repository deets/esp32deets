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


float SHT3XDIS::raw2humidity(uint16_t humidity)
{
  return float(float(humidity) * 100 / 65535.0);
}

float SHT3XDIS::raw2temperature(uint16_t temperature)
{
  return float(float(temperature) * 175.0 / 65535.0 - 45.0);
}

RawValues SHT3XDIS::raw_values()
{
  auto lock = _bus.lock();
  while(_bus.write_buffer_to_address(_address, MEASUREMENT.data(), MEASUREMENT.size()) == ESP_ERR_TIMEOUT) {}
  vTaskDelay(MEASUREMENT_TIME / portTICK_PERIOD_MS);

  std::array<uint8_t, 6> buffer;
  while(_bus.read_from_address_into_buffer(_address, buffer.data(), buffer.size()) == ESP_ERR_TIMEOUT) {}
  const auto temp = uint16_t(buffer[0] << 8 | buffer[1]);
  const auto humidity = uint16_t(buffer[3] << 8 | buffer[4]);
  return { humidity, temp };
}

Values SHT3XDIS::values()
{
  const auto raw = raw_values();
  return {
    raw2humidity(raw.humidity),
    raw2temperature(raw.temperature)
  };
}


void SHT3XDIS::reset()
{
  auto lock = _bus.lock();
  while(_bus.write_buffer_to_address(_address, RESET.data(), RESET.size()) == ESP_ERR_TIMEOUT) {}
}

void SHT3XDIS::clear()
{
  auto lock = _bus.lock();
  while(_bus.write_buffer_to_address(_address, CLEAR.data(), CLEAR.size()) == ESP_ERR_TIMEOUT) {}
}

uint16_t SHT3XDIS::status()
{
  auto lock = _bus.lock();
  while(_bus.write_buffer_to_address(_address, STATUS_.data(), STATUS_.size()) == ESP_ERR_TIMEOUT) {}
  std::array<uint8_t, 3> buffer;
  while(_bus.read_from_address_into_buffer(_address, buffer.data(), buffer.size()) == ESP_ERR_TIMEOUT) {}
  return buffer[0] << 8 | buffer[1];
}

Values Values::from_raw(const RawValues &raw)
{
  return Values {
    SHT3XDIS::raw2humidity(raw.humidity),
    SHT3XDIS::raw2temperature(raw.temperature)
  };
}

}
