// Copyright: 2022, Diez B. Roggisch, Berlin, all rights reserved

#include "flash.hpp"
#include "nvs_flash.h"

namespace deets::flash {

void init() {
  static bool initialized = false;
  if(!initialized)
  {
    initialized = true;
    nvs_flash_init();
  }
}

}
