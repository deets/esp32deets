#include "nrf24.hh"


#include <esp_timer.h>
#include <driver/uart.h>
#include <esp_log.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

// global configuration settings
#define RETRIES 15
#define RETRY_PAUSE 3 // times 250us
#define CHANNEL 124 // beyond WIFI@2.4GHz
#define SEND_TIMEOUT 50 // ms.
#define SPI_SPEED 2 * 1000*1000
#define RECEIVE_TIMEOUT_MS 5
#define START_LISTENING_TIMEOUT_US 130
#define TX_SWITCH_DELAY_US (START_LISTENING_TIMEOUT_US + 50)

// CONFIG register
#define MASK_RX_DR  0x40
#define MASK_TX_DS  0x20
#define MASK_MAX_RT 0x10
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01

// RF_SETUP register
#define POWER_0     0x00
#define POWER_1     0x02
#define POWER_2     0x04
#define POWER_3     0x06
#define SPEED_1M    0x00
#define SPEED_2M    0x08
#define SPEED_250K  0x20

// STATUS register
#define RX_DR       0x40
#define TX_DS       0x20
#define MAX_RT      0x10

// FIFO_STATUS register
#define RX_EMPTY    0x01

// constants for instructions
#define R_RX_PL_WID  0x60
#define R_RX_PAYLOAD 0x61;
#define W_TX_PAYLOAD 0xa0
#define FLUSH_TX     0xe1
#define FLUSH_RX     0xe2
#define NOP          0xff




namespace {

const size_t PAYLOAD_SIZE=32;

// hardware setup on the newjoy baseboard

const gpio_num_t CS = static_cast<gpio_num_t>(5);
const gpio_num_t CE = static_cast<gpio_num_t>(19);
const gpio_num_t MISO = static_cast<gpio_num_t>(22);
const gpio_num_t MOSI = static_cast<gpio_num_t>(23);
const gpio_num_t SCK = static_cast<gpio_num_t>(18);
const gpio_num_t SENDING_DEBUG_PIN = static_cast<gpio_num_t>(13);

class AutoStopListener
{
public:
  AutoStopListener(NRF24& nrf24) : _nrf24(nrf24) {}
  ~AutoStopListener()
  {
    _nrf24.stop_listening();
  }
private:
  NRF24& _nrf24;
};

} // end anonymous ns

#define SPI_ERROR_CHECK \
  switch(spi_res) \
  { \
  case ESP_OK: \
    res = NRF24_ERROR_OK; \
    break; \
  case ESP_ERR_INVALID_ARG: \
    res = NRF24_ERROR_INVALID_ARG; \
    break; \
  case ESP_ERR_INVALID_STATE: \
    res = NRF24_ERROR_HOST_IN_USE; \
    break; \
  case ESP_ERR_NO_MEM: \
    res = NRF24_ERROR_MALLOC; \
    break; \
  case ESP_ERR_NOT_FOUND: \
    res = NRF24_ERROR_NO_CS_SLOT; \
  default: \
    res = NRF24_ERROR_UNKNOWN; \
  } \
  if(res) \
  { \
    assert(false);                              \
  }


static void nrf24_usleep(uint32_t us)
{
  ets_delay_us(us);
}


static void nrf24_ce(uint8_t value)
{
  gpio_set_level(CE, value);
}


static void nrf24_sending(uint8_t value)
{
  gpio_set_level(SENDING_DEBUG_PIN, value);
}


uint8_t NRF24::reg_write(uint8_t reg, uint8_t value)
{
  esp_err_t res;
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  //printf("rw %i %i\n", reg, value);
  t.length = 16;
  t.tx_data[0] = 0x20 | reg; // this marks the register to be written
  t.tx_data[1] = value;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
  // according to spec, the STATUS register is
  // *always* shifted out with the first byte coming in
  return t.rx_data[0];
}


void NRF24::reg_write_bytes(uint8_t reg, const uint8_t* buf, size_t len)
{
  esp_err_t res;
  assert(len + 1 < NRF24_WORK_BUFFER_SIZE);
  //printf("rwb %i", reg);
  _tx_work_buffer[0] = 0x20 | reg;
  for(size_t i=0; i < len; ++i)
  {
    _tx_work_buffer[1 + i] = buf[i];
    //printf(" %i", buf[i]);
  }
  //printf("\n");
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8 + len * 8;
  t.tx_buffer = _tx_work_buffer;
  t.rx_buffer = _rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
}


uint8_t NRF24::reg_read(uint8_t reg)
{
  esp_err_t res;
  uint8_t ret;
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 16;
  t.tx_data[0] = reg;
  t.tx_data[1] = 0;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
  ret = t.rx_data[1];
  //printf("rr: %#2x: %#2x\n", reg, ret);
  return ret;
}


void NRF24::reg_read_bytes(uint8_t reg, uint8_t* buf, size_t len)
{
  esp_err_t res;
  assert(len + 1 < NRF24_WORK_BUFFER_SIZE);
  _tx_work_buffer[0] = reg;
  for(size_t i=0; i < len; ++i)
  {
    _tx_work_buffer[1 + i] = 0;
    _rx_work_buffer[1 + i] = 0;
  }
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8 + len * 8;
  t.tx_buffer = _tx_work_buffer;
  t.rx_buffer = _rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(_spi, &t);
  for(size_t i=0; i < len; ++i)
  {
    buf[i] = _rx_work_buffer[1 + i];
  }
  ESP_ERROR_CHECK(res);
}


void NRF24::set_power_speed(uint8_t power, uint8_t speed)
{
  uint8_t setup = reg_read(NRF24_RF_SETUP) & 0b11010001;
  reg_write(NRF24_RF_SETUP, setup | power | speed);
}


void NRF24::set_crc(uint8_t crc_length)
{
  uint8_t config = reg_read(NRF24_CONFIG) & ~(CRCO | EN_CRC | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT);
  ESP_LOGE("nrf24", "NRF24_CONFIG before CRC: %i", config);
  switch(crc_length)
  {
  case 0:
    break;
  case 1:
    config |= EN_CRC;
    break;
  case 2:
    config |= EN_CRC | CRCO;
    break;
  default:
    assert(0);
  }
  ESP_LOGE("nrf24", "NRF24_CONFIG: %i", config);
  reg_write(NRF24_CONFIG, config);
}


void NRF24::set_channel(uint8_t channel)
{
  if(channel > 125)
  {
    channel = 125;
  }
  reg_write(NRF24_RF_CH, channel);
}


void NRF24::flush_rx()
{
  esp_err_t res;
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8;
  t.tx_data[0] = FLUSH_RX;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
}


void NRF24::flush_tx()
{
  esp_err_t res;
  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8;
  t.tx_data[0] = FLUSH_TX;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
}


void NRF24::send_start(const uint8_t* payload, int payload_size)
{
  esp_err_t res;
  assert(payload_size >= 1 && payload_size <= PAYLOAD_SIZE);

  _tx_work_buffer[0] = W_TX_PAYLOAD;
  for(size_t i=0; i < payload_size; ++i)
  {
    _tx_work_buffer[1 + i] = payload[i];
  }
  // clear out all bytes after given size
  for(size_t i=payload_size; i < PAYLOAD_SIZE; ++i)
  {
    _tx_work_buffer[1 + i] = 0;
  }

  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  t.length = 8 + PAYLOAD_SIZE * 8;
  t.tx_buffer = _tx_work_buffer;
  t.rx_buffer = _rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);

  nrf24_ce(1);
  nrf24_usleep(15);  // needs to be >10us to activate transmission
  nrf24_ce(0);
}

void NRF24::dump_pipe_addresses()
{
  uint8_t buffer[5];
  memset(buffer, 0, 5);
  reg_read_bytes(NRF24_TX_ADDR, buffer, 5);
  printf("tx: %s\n", buffer);
  memset(buffer, 0, 5);
  reg_read_bytes(NRF24_RX_ADDR_P0, buffer, 5);
  printf("p0: %s\n", buffer);
  memset(buffer, 0, 5);
  reg_read_bytes(NRF24_RX_ADDR_P0 + 1, buffer, 5);
  printf("p1: %s\n", buffer);
}


NRF24::NRF24()
  : _setup(false)
{
}


int NRF24::setup(const char local_address[5])
{
  int res = 0;
  esp_err_t spi_res;

  if(_setup)
  {
    return NRF24_ERROR_ALREADY_SETUP;
  }

  gpio_pad_select_gpio(CE);
  gpio_set_level(CE, 0);
  gpio_set_direction(CE, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(SENDING_DEBUG_PIN);
  gpio_set_level(SENDING_DEBUG_PIN, 0);
  gpio_set_direction(SENDING_DEBUG_PIN, GPIO_MODE_OUTPUT);

  spi_bus_config_t buscfg = {
    MOSI, // mosi_io_num
    MISO, // miso_io_num
    SCK, // sclk_io_num
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
    0, // mode
    0, // duty_cycle_pos
    0, // cs_ena_pretrans
    0, // cs_ena_posttrans
    SPI_SPEED, // clock_speed_hz
    0, // input_delay_ns
    CS, // spics_io_num
    0, // flags
    1, // queue_size
    nullptr, // transaction_cb_t pre_cb
    nullptr // transaction_cb_t post_cb
  };

  //Initialize the SPI bus
  spi_res = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
  SPI_ERROR_CHECK;
  spi_res = spi_bus_add_device(VSPI_HOST, &devcfg, &_spi);
  SPI_ERROR_CHECK;

  // from here on, the __init__ of nrf24l01.py is lifted
  reg_write(NRF24_SETUP_AW, 0b11);
  if(reg_read(NRF24_SETUP_AW) == 0b11)
  {
    reg_write(NRF24_DYNPD, 0);
    reg_write(NRF24_SETUP_RETR, (RETRY_PAUSE << 4) | RETRIES);
    set_power_speed(POWER_3, SPEED_2M);
    set_crc(2);
    reg_write(NRF24_STATUS, RX_DR | TX_DS | MAX_RT);
    set_channel(CHANNEL);
    flush_rx();
    flush_tx();
    open_tx_pipe(local_address, PAYLOAD_SIZE);
    char fake_address[] = { 'F', 'O', 'O', 'B', 0 };
    open_rx_pipe(1, fake_address, PAYLOAD_SIZE);
    return 0;
  } else {
      teardown();
      return NRF24_ERROR_HARDWARE_NOT_RESPONDING;
  }
}


void NRF24::teardown()
{
  spi_bus_remove_device(_spi);
  spi_bus_free(VSPI_HOST);
  _setup = false;
}


void NRF24::start_listening()
{
  reg_write(NRF24_CONFIG, reg_read(NRF24_CONFIG) | PWR_UP | PRIM_RX);
  reg_write(NRF24_STATUS, RX_DR | TX_DS | MAX_RT);
  // TODO: is this necessary?
  /* if self.pipe0_read_addr is not None: */
  /*     self.reg_write_bytes(RX_ADDR_P0, self.pipe0_read_addr) */

  flush_rx();
  flush_tx();
  nrf24_ce(1);
  nrf24_sending(0);
  nrf24_usleep(START_LISTENING_TIMEOUT_US);
}


void NRF24::stop_listening()
{
  nrf24_ce(0);
  nrf24_sending(1);
  reg_write(NRF24_STATUS, RX_DR | TX_DS | MAX_RT);
  flush_rx();
  flush_tx();
  reg_write(NRF24_CONFIG, (reg_read(NRF24_CONFIG) | PWR_UP) & ~PRIM_RX);
}


int NRF24::any()
{
  int res = !(reg_read(NRF24_FIFO_STATUS) & RX_EMPTY);
  //nrf24_dump_pipe_addresses();
  return res;
}


nrf24_send_error_t NRF24::send_done()
{
  if(!(reg_read(NRF24_STATUS) & (TX_DS | MAX_RT)))
  {
    return NRF24_SEND_ERROR_NONE;
  }
  // either finished or failed: get and clear status flags, power down
  uint8_t status = reg_write(NRF24_STATUS, RX_DR | TX_DS | MAX_RT);
  reg_write(NRF24_CONFIG, reg_read(NRF24_CONFIG) & ~PWR_UP);
  if(status & TX_DS)
  {
    return NRF24_SEND_ERROR_OK;
  }
  if(status & MAX_RT)
  {
    uint8_t observe = reg_read(NRF24_OBSERVE_TX);
    if(observe != _error_info.last_observe_tx)
    {
      // we actually have a real MAX_RT reached case
      // and return that
      _error_info.last_observe_tx = observe;
      return NRF24_SEND_ERROR_MAX_RT;
    }
  }
  // the rest (which according to LOGIC!! (*sigh*)
  // should only be MAX_RT but no observable difference
  // is spurious.
  return NRF24_SEND_ERROR_SPURIOUS;
}


int64_t ticks_diff(int64_t end, int64_t start)
{
  // TODO: overrun!
  return (end - start);
}


nrf24_send_error_t NRF24::send(const uint8_t* payload, size_t payload_size)
{
  send_start(payload, payload_size);

  int64_t start = esp_timer_get_time();
  nrf24_send_error_t result = NRF24_SEND_ERROR_NONE;
  while(result == NRF24_SEND_ERROR_NONE && ticks_diff(esp_timer_get_time(), start) < SEND_TIMEOUT * 1000)
  {
    result = send_done();
  }
  switch(result)
  {
  case NRF24_SEND_ERROR_NONE:
    ++_error_info.timeout;
    break;
  case NRF24_SEND_ERROR_OK:
    ++_error_info.ok;
    break;
  case NRF24_SEND_ERROR_MAX_RT:
    ++_error_info.max_rt;
    break;
  case NRF24_SEND_ERROR_SPURIOUS:
    ++_error_info.spurious;
    break;
  }
  return result;
}


size_t NRF24::recv(unsigned char* buffer, size_t len)
{
  esp_err_t res;
  _tx_work_buffer[0] = R_RX_PAYLOAD;
  memset(_tx_work_buffer + 1, 0, PAYLOAD_SIZE);
  memset(_rx_work_buffer + 1, 0, PAYLOAD_SIZE);

  struct spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  t.length = 8 + PAYLOAD_SIZE * 8;
  t.tx_buffer = _tx_work_buffer;
  t.rx_buffer = _rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(_spi, &t);
  ESP_ERROR_CHECK(res);
  reg_write(NRF24_STATUS, RX_DR);
  size_t to_copy = PAYLOAD_SIZE;
  if(buffer)
  {
    to_copy = std::min(len, PAYLOAD_SIZE);
    memcpy(buffer, _rx_work_buffer + 1, to_copy);
  }
  return to_copy;
}


void NRF24::open_tx_pipe(const char address[5], int payload_size)
{
  // This needs to be set to the same address according to
  // the datasheed for auto-ack
  reg_write_bytes(NRF24_RX_ADDR_P0, (const uint8_t*)address, 5);
  reg_write_bytes(NRF24_TX_ADDR, (const uint8_t*)address, 5);
  reg_write(NRF24_RX_PW_P0, payload_size);
  reg_write(NRF24_EN_RXADDR, reg_read(NRF24_EN_RXADDR) | (1 << 0));
}


void NRF24::open_rx_pipe(int pipe_id, const char address[5], int payload_size)
{
  // I only allow the rx-pipe to be 1-5 because
  // the pipe 0 is always equal to the tx pipe's address
  // I also currently allow just for *one* rx pipe with
  // the full address. I don't need more, and the code
  // is not supposed to be generic.
  // So in sum, pipe_id can only be 1 ;)
  assert(1 <= pipe_id && pipe_id < 2);
  reg_write_bytes(NRF24_RX_ADDR_P0 + pipe_id, (const uint8_t*)address, 5);
  reg_write(NRF24_RX_PW_P0 + pipe_id, payload_size);
  reg_write(NRF24_EN_RXADDR, reg_read(NRF24_EN_RXADDR) | (1 << pipe_id));
}

int NRF24::wait_for_incoming_or_timeout()
{
  int64_t start_time = esp_timer_get_time();
  while(!any())
  {
    if(ticks_diff(esp_timer_get_time(), start_time) > RECEIVE_TIMEOUT_MS * 1000)
    {
      return 1;
    }
  }
  return 0;
}

// Implements the ping and subsequent message retrieval for
// a given spoke
nrf24_hub_to_spoke_error_t NRF24::hub_to_spoke(const char remote_address[5], uint8_t** buffer, size_t* len)
{
  AutoStopListener asl(*this);

  *buffer = 0;
  *len = 0;
  nrf24_hub_to_spoke_error_t result = NRF24_HUB_ERROR_OK;

  open_tx_pipe(remote_address, PAYLOAD_SIZE);

  // // workaround...
  // nrf24_send_start((const uint8_t*)"PING", 4);
  // now REALLY send
  nrf24_send_error_t send_error = send((const uint8_t*)"PING", 4);
  if(send_error != NRF24_SEND_ERROR_OK)
  {
    return NRF24_HUB_SEND_FAILED;
  }

  start_listening();

  size_t received_bytes = 0;
  uint8_t* buffer_pointer = _hub_work_buffer;
  uint8_t packets_left = 1;
  while(packets_left)
  {
    if(wait_for_incoming_or_timeout())
    {
      return NRF24_HUB_RX_TIMEOUT;
    }
    recv(0, 0);
    // the rx_buffer has a leading padding byte,
    // so we have to skip two, not just one, for
    // the packet length
    uint8_t packet_length = _rx_work_buffer[2];
    // same reason here - skip one byte more
    packets_left = _rx_work_buffer[1];
    received_bytes += packet_length;
    if(received_bytes > NRF24_HUB_WORK_BUFFER_SIZE)
    {
      return NRF24_HUB_PAYLOAD_TOO_LONG;
    }
    memcpy(buffer_pointer, _rx_work_buffer + 3, packet_length);
    buffer_pointer += packet_length;
  }

  *len = received_bytes;
  *buffer = _hub_work_buffer;

  return result;
}


void NRF24::clear_error_info()
{
  memset(&_error_info, 0, sizeof(nrf24_error_info_t));
  // this is needed according to section 7.4.2
  // to reset OBSERVE_TX
  set_channel(CHANNEL);
}


nrf24_error_info_t NRF24::error_info()
{
  return _error_info;
}

#define PROTOCOL_HEADER_LENGTH 2
#define PAYLOAD_AVAILABLE (PAYLOAD_SIZE - PROTOCOL_HEADER_LENGTH)

nrf24_spoke_to_hub_error_t NRF24::spoke_to_hub_send(const uint8_t * buffer, size_t len)
{
  nrf24_spoke_to_hub_error_t res = NRF24_SPOKE_ERROR_OK;

  stop_listening();
  // give the TX time to switch to become RX
  nrf24_usleep(TX_SWITCH_DELAY_US);

  int packets_to_send = len / PAYLOAD_AVAILABLE + (
    (len % PAYLOAD_AVAILABLE > 0) ? 1 : 0);

  //printf("for %i bytes sending %i packets\n", len, packets_to_send);

  for(int packet=0; packet < packets_to_send; ++packet)
  {
    uint8_t packet_buffer[PAYLOAD_SIZE];
    uint8_t to_copy = std::min(len, PAYLOAD_AVAILABLE);

    //printf("to_copy %i\n", to_copy);

    packet_buffer[0] = packets_to_send - packet - 1;
    packet_buffer[1] = to_copy;

    uint8_t* dest = packet_buffer + PROTOCOL_HEADER_LENGTH;
    memcpy(dest, buffer, to_copy);
    buffer += to_copy;
    len -= to_copy;
    nrf24_send_error_t send_error = send(packet_buffer, to_copy + PROTOCOL_HEADER_LENGTH);

    switch(send_error)
    {
      // this means a timetout... I bail out!
    case NRF24_SEND_ERROR_NONE:
    case NRF24_SEND_ERROR_MAX_RT:
    case NRF24_SEND_ERROR_SPURIOUS:
      res = NRF24_SPOKE_SEND_FAILED;
      goto exit;
    case NRF24_SEND_ERROR_OK:
      break;
    }
  }

exit:
  start_listening();
  return res;
}
