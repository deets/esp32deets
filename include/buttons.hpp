// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include "hal/gpio_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <functional>
#include <initializer_list>
#include <variant>

namespace deets::buttons {


enum class pull_e { NONE, UP, DOWN };
enum class irq_e { NONE, POS, NEG, ANY };

struct event_group_config_t
{
  EventGroupHandle_t group;
  EventBits_t bits;
};

using callback_config_t =
    std::variant<deets::buttons::event_group_config_t, std::monostate>;

struct gpio_config_t
{
  gpio_num_t num;
  pull_e pull;
  irq_e irq;
  int debounce; // in microseconds
  callback_config_t callback_config;
};

void setup_pin(const gpio_config_t& pin);
void setup(std::initializer_list<gpio_config_t> config);

void register_button_callback(gpio_num_t num,
                              std::function<void(gpio_num_t)>);

} // namespace deets::iobuttons
