// Copyright: 2022, Diez B. Roggisch, Berlin, all rights reserved

#include "deets/eventloop.hpp"

#include "esp_event.h"

namespace deets::eventloop {

void init()
{
  static bool initialized = false;
  if(!initialized)
  {
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initialized = true;
  }
}

} // namespace deets::eventloop
