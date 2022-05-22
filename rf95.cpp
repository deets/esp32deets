#include "rf95.hpp"
#include "buttons.hpp"
#include "driver/spi_common.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <cstring>
#include <initializer_list>
#include <sys/types.h>

#define TAG "rf95"


#define RH_RF95_MAX_POWER 0x70
#define RH_RF95_PA_DAC_DISABLE 0x04
#define RH_RF95_PA_DAC_ENABLE  0x07
#define RH_RF95_PA_SELECT 0x80
#define IRQ_BIT (1 << 0)

// The crystal oscillator frequency of the module
#define RH_RF95_FXOSC 32000000.0
// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)

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

namespace {

uint8_t operator | (const RF95::op_reg& left, const RF95::op_reg& right)
{
  return uint8_t(int(left) | int(right));
}

}

RF95::RF95(spi_host_device_t spi_host, gpio_num_t cs, gpio_num_t sck,
           gpio_num_t mosi, gpio_num_t miso, int speed, gpio_num_t irq)
  : _spi_host(spi_host)
{
  int res;
  esp_err_t spi_res;

  gpio_pad_select_gpio(cs);
  gpio_set_level(cs, 0);
  gpio_set_direction(cs, GPIO_MODE_OUTPUT);

  spi_bus_config_t buscfg = {
    mosi, // mosi_io_num
    miso, // miso_io_num
    sck, // sclk_io_num
    -1, // quadwp_io_num
    -1, // quadhd_io_num
    -1, // data4_io_num
    -1, // data5_io_num
    -1, // data6_io_num
    -1, // data7_io_num
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

  _irq_event_group = xEventGroupCreate();

  deets::buttons::setup_pin(
    {
      irq,
      deets::buttons::pull_e::DOWN,
      deets::buttons::irq_e::POS,
      0
    }
    );
  deets::buttons::register_button_callback(
    gpio_num_t(26),
    [this](gpio_num_t) {
      xEventGroupSetBits(_irq_event_group, IRQ_BIT);
    }
    );


  setup();
}


RF95::~RF95() {}


void RF95::setup()
{
  // This is based on the RadioHead library
  // Sleep, and set LoRa operation mode
  const auto desired_mode = op_reg::LORA | op_reg::SLEEP;
  reg_write(register_t::OP_MODE, desired_mode);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  assert(desired_mode == reg_read(register_t::OP_MODE));

  // set up FIFO
  // we configure so that we can use the entire 256 byte FIFO for either receive
  // or transmit, but not both at the same time
  for(const auto reg : { register_t::FIFO_TX_BASE, register_t::FIFO_RX_BASE})
  {
    reg_write(reg, 0);
  }
  mode(IDLE);

  modem_config({ 0x72, 0x74, 0x04});
  preamble_length(8);
  frequency(868.0);
  tx_power(13);
}


void RF95::send(const uint8_t *buffer, size_t len, int timeout) {
  mode(IDLE);
  // if (!waitCAD())
  //   return false;  // Check channel activity

  // Position at the beginning of the FIFO
  reg_write(register_t::FIFO_ADDR_PTR, 0);
  // The headers
  reg_write(register_t::FIFO, 0xAA);
  reg_write(register_t::FIFO, 0x55);
  reg_write(register_t::FIFO, 0xFF);
  reg_write(register_t::FIFO, 0x33);
  // The message data
  //spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);

  xEventGroupClearBits(
    _irq_event_group,
    IRQ_BIT
    );
  reg_write(register_t::PAYLOAD_LENGTH, 4);
  mode(TX);
  const auto bits = xEventGroupWaitBits(
    _irq_event_group,
    IRQ_BIT,
    pdFALSE,
    pdFALSE,
    timeout / portTICK_PERIOD_MS);

  if(!bits)
  {
    ESP_LOGE(TAG, "IRQ didn't arrive in time");
  }
  else
  {
    ESP_LOGI(TAG, "TX acknowledged by IRQ");
  }
}


void RF95::modem_config(const std::array<uint8_t, 3>& config)
{
  reg_write(register_t::MODEM_CONFIG_1, config[0]);
  reg_write(register_t::MODEM_CONFIG_2, config[1]);
  reg_write(register_t::MODEM_CONFIG_3, config[2]);
}


void RF95::preamble_length(uint16_t bytes)
{
  reg_write(register_t::PREAMBLE_MSB, uint8_t(bytes >> 8));
  reg_write(register_t::PREAMBLE_LSB, uint8_t(bytes));
}


void RF95::frequency(float centre)
{
  uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
  reg_write(register_t::FREQ_MSB, uint8_t(frf >> 16));
  reg_write(register_t::FREQ_MID, uint8_t(frf >> 8));
  reg_write(register_t::FREQ_LSB, uint8_t(frf));
}


void RF95::tx_power(int power)
{
  // Apparently my module only uses the high power
  // output.
  assert(power >= 2 && power <= 20);
  if(power > 17)
  {
    // According to RadioHead, the DAC enabling gives
    // us around 3 DB.
    reg_write(register_t::PA_DAC, RH_RF95_PA_DAC_ENABLE);
    power -= 3;
  }
  else
  {
    reg_write(register_t::PA_DAC, RH_RF95_PA_DAC_DISABLE);
  }
  reg_write(register_t::PA_CONFIG, RH_RF95_PA_SELECT | (power-2));
}


void RF95::mode(mode_t mode)
{
  // I'm a bit suprised this works, because
  // I would have thought overwriting the LoRa
  // bit like this is a bad idea. I presume it works
  // as long as we don't sleep?
  switch(mode)
  {
  case IDLE:
    reg_write(register_t::OP_MODE, uint8_t(op_reg::STDBY));
    break;
  case TX:
    // According to table 18, this should give us TX done on DIO0, the only pin
    // broken out to the ESP.
    reg_write(register_t::DIO_MAPPING_1, 0x01 << 6);
    // Clear IRQs
    reg_write(register_t::IRQ_FLAGS, 0xFF);
    reg_write(register_t::OP_MODE, uint8_t(op_reg::TX));
    break;
  }
}


uint8_t RF95::reg_read(register_t register_)
{
  esp_err_t res;
  uint8_t ret;
  struct spi_transaction_t t;
  std::memset(&t, 0, sizeof(t));
  t.length = 16;
  t.tx_data[0] = uint8_t(register_) & 0x7f; // force read
  t.tx_data[1] = 0;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
  ret = t.rx_data[1];
  return ret;
}

void RF95::reg_write(register_t register_, uint8_t value)
{
  esp_err_t res;
  struct spi_transaction_t t;
  std::memset(&t, 0, sizeof(t));
  t.length = 16;
  t.tx_data[0] = (uint8_t(register_) & 0x7f) | 0x80; // force write
  t.tx_data[1] = value;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
}
