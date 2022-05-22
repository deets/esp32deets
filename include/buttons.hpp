// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include "hal/gpio_types.h"

#include <functional>
#include <initializer_list>

namespace deets::buttons {


enum class pull_e { NONE, UP, DOWN };
enum class irq_e { NONE, POS, NEG, ANY };

struct gpio_config_t
{
  gpio_num_t num;
  pull_e pull;
  irq_e irq;
  int debounce; // in microseconds
};

void setup_pin(const gpio_config_t& pin);
void setup(std::initializer_list<gpio_config_t> config);

void register_button_callback(gpio_num_t num,
                              std::function<void(gpio_num_t)>);

} // namespace deets::iobuttons
