/*
 * Fabien Bonneval fabien.bonneval[at]gmail.com
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file peripherals/ccs811_i2c.h
 * @brief Sensor driver for CCS811 sensor via I2C
 *
 */

#ifndef CCS811_I2C_H
#define CCS811_I2C_H

#include "mcu_periph/i2c.h"
#include "peripherals/ccs811_regs.h"

typedef enum {
    // wait a bit to be sure the sensor is ready
    CCS811_HOLD,
    // write to APP_START (assume firmware is present)
    CCS811_UNINIT,
    // write mode to MEAS_MODE
    CCS811_BOOTED,
    // request data
    CCS811_MEAS_MODE_OK,
    // read data. If status ok, goto WAIT, if noK, request data again (wait a bit?), and stay in that state
    CCS811_READ_DATA,
    // when 1s has passed, request data and goto READ_DATA
    CCS811_WAIT,
    // if some error happens. TODO handle it.
    CCS811_ERROR,
} Ccs811State;

typedef struct {
  uint16_t eCo2;
  uint16_t eTvoc;
  uint8_t status;
  uint8_t errorId;
  uint8_t rawData[2];
} __attribute__((__packed__)) RxData_t;

typedef struct {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  
  Ccs811State state;

  bool data_available;

  // data
  //Css811Version versions;
  uint8_t ccs811_status;
  //uint16_t baseLine;
  
  float resistance;
  uint16_t	co2;
  uint16_t	tvoc;
  struct {
    uint8_t  current:6;
    uint16_t adc:10;
  } raw;
} Ccs811_I2c;


extern void ccs811_i2c_init(Ccs811_I2c *ccs, struct i2c_periph *i2c_p, uint8_t addr);
extern void ccs811_i2c_periodic(Ccs811_I2c *ccs);
extern void ccs811_i2c_event(Ccs811_I2c *ccs);


#endif

