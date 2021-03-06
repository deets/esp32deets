// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include <cinttypes>

class I2C;

namespace sht3xdis {

struct RawValues
{
  uint16_t humidity;
  uint16_t temperature;
};

struct Values
{
  float humidity;
  float temperature;
};

class SHT3XDIS
{
public:

  static bool valid_address(uint8_t address);

  SHT3XDIS(I2C& bus, uint8_t address);

  RawValues raw_values();
  Values values();
  void reset();
  void clear();
  uint16_t status();

private:

  I2C& _bus;
  uint8_t _address;
};

}
