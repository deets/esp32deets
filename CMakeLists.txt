# This file has a problem, see:
#
#  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#component-requirements
#
# It can't build a requirements list depending on the config choices. Need to figure out what todo about that.

set(
  srcs
  )

if(CONFIG_DEETS_USE_WIFI)
  list(APPEND srcs
    eventloop.cpp
    smartconfig.cpp
    wifi.cpp
    flash.cpp
  )
endif()

if(CONFIG_DEETS_USE_BUTTONS)
  list(APPEND srcs
    buttons.cpp
  )
endif()

if(CONFIG_DEETS_USE_U8G2)
  list(APPEND srcs u8g2_esp32_hal.c)
endif()

if(CONFIG_DEETS_USE_MPU6050)
  list(APPEND srcs mpu6050.cpp madgwick.cpp)
endif()

if(CONFIG_DEETS_USE_I2C)
  list(APPEND srcs i2c.cpp)
endif()

if(CONFIG_DEETS_USE_TCA9548A)
  list(APPEND srcs tca9548a.cpp)
endif()

if(CONFIG_DEETS_USE_NRF24)
  list(APPEND srcs nrf24.cpp)
endif()

if(CONFIG_DEETS_USE_RF95)
  list(APPEND srcs rf95.cpp)
endif()

if(CONFIG_DEETS_USE_DRV2605)
  list(APPEND srcs drv2605.cpp)
endif()

if(CONFIG_DEETS_USE_SHT3XDIS)
  list(APPEND srcs sht3xdis.cpp)
endif()

if(CONFIG_DEETS_USE_HTTP)
  list(APPEND srcs http.cpp)
  list(APPEND requirements esp_http_server)
endif()


idf_component_register(
  .
  SRCS ${srcs}
  INCLUDE_DIRS "include"
  REQUIRES "driver"
  )

target_compile_definitions(${COMPONENT_LIB} PUBLIC U8X8_WITH_USER_PTR)
