// Copyright: 2019, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <driver/spi_master.h>

#include <stddef.h>
#include <stdint.h>

// nRF24L01+ registers
#define NRF24_CONFIG      0x00
#define NRF24_EN_AA       0x01
#define NRF24_EN_RXADDR   0x02
#define NRF24_SETUP_AW    0x03
#define NRF24_SETUP_RETR  0x04
#define NRF24_RF_CH       0x05
#define NRF24_RF_SETUP    0x06
#define NRF24_STATUS      0x07
#define NRF24_OBSERVE_TX  0x08
#define NRF24_RX_ADDR_P0  0x0a
#define NRF24_TX_ADDR     0x10
#define NRF24_RX_PW_P0    0x11
#define NRF24_FIFO_STATUS 0x17
#define NRF24_DYNPD	    0x1c

#define NRF24_WORK_BUFFER_SIZE 64
#define NRF24_HUB_WORK_BUFFER_SIZE 512


enum nrf24_error_t {
  NRF24_ERROR_OK,
  NRF24_ERROR_ALREADY_SETUP,
  NRF24_ERROR_MALLOC,
  NRF24_ERROR_INVALID_ARG,
  NRF24_ERROR_HOST_IN_USE,
  NRF24_ERROR_NO_CS_SLOT,
  NRF24_ERROR_HARDWARE_NOT_RESPONDING,
  NRF24_ERROR_UNKNOWN
};

enum nrf24_hub_to_spoke_error_t
{
  NRF24_HUB_ERROR_OK,
  NRF24_HUB_SEND_FAILED,
  NRF24_HUB_RX_TIMEOUT,
  NRF24_HUB_PAYLOAD_TOO_LONG
};

enum nrf24_spoke_to_hub_error_t
{
  NRF24_SPOKE_ERROR_OK,
  NRF24_SPOKE_SEND_FAILED,
};


enum nrf24_send_error_t
{
  NRF24_SEND_ERROR_NONE = -1,
  NRF24_SEND_ERROR_OK = 1,
  NRF24_SEND_ERROR_MAX_RT = 2,
  NRF24_SEND_ERROR_SPURIOUS = 3
};

struct nrf24_error_info_t
{
  uint8_t last_observe_tx;
  uint32_t timeout;
  uint32_t ok;
  uint32_t max_rt;
  uint32_t spurious;
};

class NRF24
{
public:
  NRF24();

  int setup(const char local_address[5]);
  uint8_t reg_read(uint8_t register);
  void open_rx_pipe(int number, const char local_address[5], int payload_size);
  void open_tx_pipe(const char remote_address[5], int payload_size);
  void teardown();
  void start_listening();
  void stop_listening();
  int any();
  void clear_error_info();
  nrf24_error_info_t error_info();
  nrf24_send_error_t send(const uint8_t* payload, size_t payload_length);
  size_t recv(unsigned char* buffer, size_t len);

  nrf24_hub_to_spoke_error_t hub_to_spoke(const char remote_address[5], uint8_t** buffer, size_t* len);

  nrf24_spoke_to_hub_error_t spoke_to_hub_send(const uint8_t * buffer, size_t len);

private:
  uint8_t reg_write(uint8_t reg, uint8_t value);
  void reg_write_bytes(uint8_t reg, const uint8_t* buf, size_t len);
  void reg_read_bytes(uint8_t reg, uint8_t* buf, size_t len);
  void set_power_speed(uint8_t power, uint8_t speed);
  void set_crc(uint8_t crc_length);
  void set_channel(uint8_t channel);
  void flush_rx();
  void flush_tx();
  void dump_pipe_addresses();
  void send_start(const uint8_t* payload, int payload_size);
  nrf24_send_error_t send_done();
  int wait_for_incoming_or_timeout();

  spi_device_handle_t _spi;
  uint8_t _tx_work_buffer[NRF24_WORK_BUFFER_SIZE];
  uint8_t _rx_work_buffer[NRF24_WORK_BUFFER_SIZE];
  uint8_t _hub_work_buffer[NRF24_HUB_WORK_BUFFER_SIZE];
  nrf24_error_info_t _error_info;

  bool _setup;

};
