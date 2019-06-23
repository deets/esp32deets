// Copyright: 2018, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include "sprite.h"

#include "driver/spi_master.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t* frame;
  uint8_t*  raw_frame;
  int dc_pin;
  int rst_pin;
  int dc_level;
  int width, height;
  int frame_byte_size;
  spi_device_handle_t spi;
} ssd1306_display_t;


int ssd1306_init_static(
  ssd1306_display_t *ssd1306_display,
  int cs, int clk, int mosi,
  int dc, int rst, int spi_speed
  );

void ssd1306_update(ssd1306_display_t* display);

void ssd1306_clear(ssd1306_display_t* display);

void ssd1306_draw_pixel(ssd1306_display_t* display, int x, int y);

void ssd1306_draw_vertical_line(ssd1306_display_t* display, int x, int y, int y2);
void ssd1306_draw_horizontal_line(ssd1306_display_t* display, int x, int x2, int y);

void ssd1306_blit(ssd1306_display_t* display, const sprite_t* const sprite, int x, int y);

#ifdef __cplusplus
}
#endif
