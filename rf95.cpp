#include "rf95.hpp"
#include "driver/spi_common.h"

#include <esp_log.h>

#include <cstring>

#define TAG "rf95"

#define SPI_ERROR_CHECK \
  switch(spi_res) \
  { \
  case ESP_OK: \
    res = 0; \
    break; \
  default: \
  res = 1; \
  } \
  if(res) \
  { \
    ESP_LOGE(TAG, "SPI ERROR: %x", spi_res); \
    assert(false);                              \
  }


RF95::RF95(spi_host_device_t spi_host, gpio_num_t cs, gpio_num_t sck,
           gpio_num_t mosi, gpio_num_t miso, int speed)
  : _spi_host(spi_host)
  , _cs(cs)
{
  int res;
  esp_err_t spi_res;

  gpio_pad_select_gpio(_cs);
  gpio_set_level(_cs, 0);
  gpio_set_direction(_cs, GPIO_MODE_OUTPUT);

  spi_bus_config_t buscfg = {
    mosi, // mosi_io_num
    miso, // miso_io_num
    sck, // sclk_io_num
    -1, // quadwp_io_num
    -1, // quadhd_io_num
    0, // max_transfer_sz
    0, // flags
    0 // intr_flags
  };

  spi_device_interface_config_t devcfg = {
    0, // command_bits
    0, // address_bits
    0, // dummy_bits
    1, // mode
    0, // duty_cycle_pos
    0, // cs_ena_pretrans
    0, // cs_ena_posttrans
    speed, // clock_speed_hz
    0, // input_delay_ns
    cs, // spics_io_num
    0, // flags
    1, // queue_size
    nullptr, // transaction_cb_t pre_cb
    nullptr // transaction_cb_t post_cb
  };

  //Initialize the SPI bus
  spi_res = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_DISABLED);
  SPI_ERROR_CHECK;
  spi_res = spi_bus_add_device(_spi_host, &devcfg, &_spi);
  SPI_ERROR_CHECK;
}

RF95::~RF95() {}

uint8_t RF95::reg_read(uint8_t register_)
{
  esp_err_t res;
  uint8_t ret;
  struct spi_transaction_t t;
  std::memset(&t, 0, sizeof(t));
  t.length = 16;
  t.tx_data[0] = register_ & 0x7f; // force read
  t.tx_data[1] = 0;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
  ret = t.rx_data[1];
  return ret;
}

void RF95::reg_write(uint8_t register_, uint8_t value)
{
  esp_err_t res;
  uint8_t ret;
  struct spi_transaction_t t;
  std::memset(&t, 0, sizeof(t));
  t.length = 16;
  t.tx_data[0] = (register_ & 0x7f) | 0x80; // force write
  t.tx_data[1] = value;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
}
