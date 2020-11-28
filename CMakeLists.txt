idf_component_register(
  .
  SRCS
  drv2605.cpp
  i2c.cpp
  mpu6050.cpp
  nrf24.cpp
  ssd1306.c
  u8g2_esp32_hal.c
  wifi.cpp
  INCLUDE_DIRS "include"
  REQUIRES u8g2
  )
