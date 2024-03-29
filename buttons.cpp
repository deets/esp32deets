// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#include "buttons.hpp"
#include "esp_err.h"

#include <soc/gpio_struct.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <esp_event_base.h>
#include <esp_log.h>

#include <unordered_map>
#include <stdint.h>
#include <list>
#include <map>


namespace deets::buttons {

namespace {

#define TAG "buttons"

ESP_EVENT_DEFINE_BASE(DEETS_BUTTON_EVENTS);


std::unordered_map<gpio_num_t, gpio_config_t> s_button_configs;

std::map<gpio_num_t, std::list<std::function<void(gpio_num_t)>>> s_button_callbacks;
std::unordered_map<int, uint64_t> s_last;

void IRAM_ATTR gpio_isr_handler(void* arg)
{
  const auto pin = gpio_num_t((int)arg);
  const auto& button_config = s_button_configs[pin];
  int64_t ts = esp_timer_get_time();
  if(s_last.count(pin) && s_last[pin] + button_config.debounce > ts)
  {
    return;
  }
  s_last[pin] = ts;
  if(const auto event_group_config = std::get_if<event_group_config_t>(&button_config.callback_config))
  {
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xEventGroupSetBitsFromISR(
                         event_group_config->group,
                         event_group_config->bits,
                         &xHigherPriorityTaskWoken );

     // Was the message posted successfully?
     if( xResult == pdPASS )
     {
         portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
     }
  }
  else
  {
    esp_event_post(
      DEETS_BUTTON_EVENTS, pin, nullptr, 0, 0);
  }
}

void s_button_event_handler(void *event_handler_arg,
                            esp_event_base_t event_base, int32_t button,
                            void *event_data)
{
  ESP_LOGD(TAG, "s_button_event_handler");
  const auto button_num = static_cast<gpio_num_t>(button);
  for(auto& cb : s_button_callbacks[button_num])
  {
    cb(button_num);
  }
}

void register_isr_handler()
{
  static bool registered = false;
  // install global GPIO ISR handler
  if(!registered)
  {
    ESP_LOGD(TAG, "Enable ISR service");
    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                      DEETS_BUTTON_EVENTS,
                      ESP_EVENT_ANY_ID,
                      s_button_event_handler, nullptr, nullptr));
    registered = true;
  }
}

} // end ns anonymous


void setup_pin(const gpio_config_t& pin)
{
  register_isr_handler();
  ESP_LOGD(TAG, "Setup pin %i", pin.num);
  gpio_set_direction(pin.num, gpio_mode_t(GPIO_MODE_DEF_INPUT));
  switch(pin.pull)
  {
  case pull_e::NONE:
    gpio_pullup_dis(pin.num);
    gpio_pulldown_dis(pin.num);
    break;
  case pull_e::UP:
    gpio_pullup_en(pin.num);
    break;
  case pull_e::DOWN:
    gpio_pulldown_en(pin.num);
    break;
  }
  switch(pin.irq)
  {
  case irq_e::NONE:
    break;
  case irq_e::POS:
    ESP_ERROR_CHECK(gpio_set_intr_type(pin.num, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin.num, gpio_isr_handler, (void*)pin.num));
    break;
  case irq_e::NEG:
    ESP_ERROR_CHECK(gpio_set_intr_type(pin.num, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin.num, gpio_isr_handler, (void*)pin.num));
    break;
  case irq_e::ANY:
    ESP_ERROR_CHECK(gpio_set_intr_type(pin.num, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin.num, gpio_isr_handler, (void*)pin.num));
    break;
  }
  //fill map to avoid allocates in ISR
  s_last[pin.num] = esp_timer_get_time();
  s_button_configs[pin.num] = pin;
}

void setup(std::initializer_list<gpio_config_t> config)
{
  for(const auto pin : config)
  {
    setup_pin(pin);
  }
}

void register_button_callback(gpio_num_t e,
                              std::function<void(gpio_num_t)> cb)
{
  if(std::get_if<std::monostate>(&s_button_configs[e].callback_config))
  {
    s_button_callbacks[e].push_back(cb);
  }
  else
  {
    ESP_LOGE(TAG, "Tried setting a callback on an event group button");
  }
}

} // namespace deets::buttons
