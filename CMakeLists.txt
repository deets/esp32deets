set(
  srcs
  wifi.cpp
  eventloop.cpp
  flash.cpp
  smartconfig.cpp
  buttons.cpp
  )

set(requirements nvs_flash)

if(CONFIG_DEETS_USE_U8G2)
  list(APPEND srcs u8g2_esp32_hal.c ssd1306.c)
  # APPEND requirements u8g2
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
  REQUIRES "esp_http_server ${requirements}"
  )
