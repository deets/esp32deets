// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once
#include <sdkconfig.h>
#ifndef CONFIG_DEETS_USE_DRV2605
#error "CONFIG_DEETS_USE_DRV2605 not set!"
#endif

#include "i2c.hh"

class DRV2605
{
public:
  DRV2605(I2CHost& i2c);
  void set_waveform(uint8_t slot, uint8_t w);
  void select_library(uint8_t lib);
  void go();
  void stop();
  void set_mode(uint8_t mode);
  void set_realtime_value(uint8_t rtp);
  void use_erm();
  void use_lra();

private:
  void write_register(uint8_t r, uint8_t v);
  uint8_t read_register(uint8_t r);

  I2CHost& _i2c;

};
