set(
  srcs
  wifi.cpp
  )

if(CONFIG_DEETS_USE_U8G2)
  list(APPEND srcs u8g2_esp32_hal.c ssd1306.c)
endif()

if(CONFIG_DEETS_USE_MPU6050)
  list(APPEND srcs mpu6050.cpp madgwick.cpp)
endif()

if(CONFIG_DEETS_USE_I2C)
  list(APPEND srcs i2c.cpp)
endif()

if(CONFIG_DEETS_USE_NRF24)
  list(APPEND srcs nrf24.cpp)
endif()

if(CONFIG_DEETS_USE_DRV2605)
  list(APPEND srcs drv2605.cpp)
endif()


idf_component_register(
  .
  SRCS ${srcs}
  INCLUDE_DIRS "include"
  REQUIRES u8g2
  )
