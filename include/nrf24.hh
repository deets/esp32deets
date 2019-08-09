// Copyright: 2019, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
  NRF24_ERROR_OK,
  NRF24_ERROR_ALREADY_SETUP,
  NRF24_ERROR_MALLOC,
  NRF24_ERROR_INVALID_ARG,
  NRF24_ERROR_HOST_IN_USE,
  NRF24_ERROR_NO_CS_SLOT,
  NRF24_ERROR_HARDWARE_NOT_RESPONDING,
  NRF24_ERROR_UNKNOWN
} nrf24_error_t;

typedef enum
{
  NRF24_HUB_ERROR_OK,
  NRF24_HUB_SEND_FAILED,
  NRF24_HUB_RX_TIMEOUT,
  NRF24_HUB_PAYLOAD_TOO_LONG
} nrf24_hub_to_spoke_error_t;

typedef enum
{
  NRF24_SPOKE_ERROR_OK,
  NRF24_SPOKE_SEND_FAILED,
} nrf24_spoke_to_hub_error_t;


typedef enum
{
  NRF24_SEND_ERROR_NONE = -1,
  NRF24_SEND_ERROR_OK = 1,
  NRF24_SEND_ERROR_MAX_RT = 2,
  NRF24_SEND_ERROR_SPURIOUS = 3
} nrf24_send_error_t;

typedef struct
{
  uint8_t last_observe_tx;
  uint32_t timeout;
  uint32_t ok;
  uint32_t max_rt;
  uint32_t spurious;
} nrf24_error_info_t;

int nrf24_setup(const char local_address[5]);
uint8_t nrf24_reg_read(uint8_t register);
void nrf24_open_rx_pipe(int number, const char local_address[5], int payload_size);
void nrf24_open_tx_pipe(const char remote_address[5], int payload_size);
void nrf24_teardown();
void nrf24_start_listening();
void nrf24_stop_listening();
int nrf24_any();
void nrf24_clear_error_info();
nrf24_error_info_t nrf24_error_info();
nrf24_send_error_t nrf24_send(const uint8_t* payload, size_t payload_length);
size_t nrf24_recv(unsigned char* buffer, size_t len);

nrf24_hub_to_spoke_error_t nrf24_hub_to_spoke(const char remote_address[5], uint8_t** buffer, size_t* len);

nrf24_spoke_to_hub_error_t nrf24_spoke_to_hub_send(const uint8_t * buffer, size_t len);
