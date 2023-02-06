#pragma once

#include "mcu_periph/spi.h"

#include <stdbool.h>
#include <math.h>

#define SPI_BUFFER_SIZE 8

enum amt22_request_t {
  AMT22_READ_POSITION,
  AMT22_RESET,
  AMT22_SET_ZERO,
  AMT22_READ_TURNS,
  AMT22_NONE,
};

struct amt22_t {
  struct spi_periph *periph;
  struct spi_transaction trans;
  volatile uint8_t spi_input_buf[SPI_BUFFER_SIZE];
  volatile uint8_t spi_output_buf[SPI_BUFFER_SIZE];


  enum amt22_request_t request;
  int16_t position;
  uint16_t offset;
  int16_t turns;
  bool flag_offset;
  float angle_rad;
  // timestamp ?
};

void amt22_init(struct amt22_t *amt, struct spi_periph *periph, uint8_t slave_idx);

void amt22_event(struct amt22_t *amt);

void amt22_request(struct amt22_t *amt,  enum amt22_request_t request);

bool amt22_checkbit(uint16_t data);