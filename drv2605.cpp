// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include "drv2605.hh"
#include "drv2605-defs.hh"

#include <esp_log.h>


DRV2605::DRV2605(I2CHost& i2c)
  : _i2c(i2c)
{
  uint8_t id = i2c.read_byte_from_register(DRV2605_ADDR, DRV2605_REG_STATUS);
  ESP_LOGI("drv2605", "id: %x", id);

  write_register(DRV2605_REG_MODE, 0x00); // out of standby

  write_register(DRV2605_REG_RTPIN, 0x00); // no real-time-playback

  write_register(DRV2605_REG_WAVESEQ1, 1); // strong click
  write_register(DRV2605_REG_WAVESEQ2, 0); // end sequence

  write_register(DRV2605_REG_OVERDRIVE, 0); // no overdrive

  write_register(DRV2605_REG_SUSTAINPOS, 0);
  write_register(DRV2605_REG_SUSTAINNEG, 0);
  write_register(DRV2605_REG_BREAK, 0);
  write_register(DRV2605_REG_AUDIOMAX, 0x64);

  // ERM open loop

  // turn off N_ERM_LRA
  write_register(DRV2605_REG_FEEDBACK,
                 read_register(DRV2605_REG_FEEDBACK) & 0x7F);
  // turn on ERM_OPEN_LOOP
  write_register(DRV2605_REG_CONTROL3,
                 read_register(DRV2605_REG_CONTROL3) | 0x20);

  set_mode(DRV2605_MODE_INTTRIG);
}

void DRV2605::write_register(uint8_t r, uint8_t v)
{
  _i2c.write_byte_to_register(DRV2605_ADDR, r, v);
}

uint8_t DRV2605::read_register(uint8_t r)
{
  return _i2c.read_byte_from_register(DRV2605_ADDR, r);
}

void DRV2605::set_waveform(uint8_t slot, uint8_t w) {
  write_register(DRV2605_REG_WAVESEQ1 + slot, w);
}

void DRV2605::select_library(uint8_t lib) {
  write_register(DRV2605_REG_LIBRARY, lib);
}

void DRV2605::go() { write_register(DRV2605_REG_GO, 1); }

void DRV2605::stop() { write_register(DRV2605_REG_GO, 0); }

void DRV2605::set_mode(uint8_t mode) {
  write_register(DRV2605_REG_MODE, mode);
}

void DRV2605::set_realtime_value(uint8_t rtp) {
  write_register(DRV2605_REG_RTPIN, rtp);
}

void DRV2605::use_erm() {
  write_register(DRV2605_REG_FEEDBACK,
                 read_register(DRV2605_REG_FEEDBACK) & 0x7F);
}

void DRV2605::use_lra() {
  write_register(DRV2605_REG_FEEDBACK,
                 read_register(DRV2605_REG_FEEDBACK) | 0x80);
}
