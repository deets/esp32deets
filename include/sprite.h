// Copyright: 2018, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <stdint.h>
#include <stdlib.h>

/*
 * Sprites
 *
 * Sprites can only be 32 pixels wide.
 * They support a hotspot that is subracted
 * from their position so that they are centered
 * around that.
 *
 * The modulo-parameter allows to skip words
 * when reading the sprite-data. This allows to
 * define sprites based of a bigger block.
 */
typedef struct {
  int height; // can be arbitrary
  int hotspot_x;
  int hotspot_y;
  int image_modulo;
  int mask_modulo;
  uint32_t* image;
  uint32_t* mask;
} sprite_t;
