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
 * @file peripherals/ccs811_i2c.c
 * @brief Sensor driver for CCS811 sensor via I2C
 *
 */
 
#include "peripherals/ccs811_i2c.h"
#include "mcu_periph/gpio.h"
#include "generated/airframe.h"
#include <string.h>

static bool ccs811I2CTransmit(Ccs811_I2c *ccsp, size_t txsize, size_t rxsize);


void ccs811_i2c_init(Ccs811_I2c *ccsp, struct i2c_periph *i2c_p, uint8_t addr)
{
  // set i2c_peripheral
  ccsp->i2c_p = i2c_p;
  ccsp->i2c_trans.slave_addr = addr;
  ccsp->i2c_trans.status = I2CTransDone;  // set initial status: Done
  ccsp->data_available = false;
  ccsp->state = CCS811_HOLD;
}


void ccs811_i2c_periodic(Ccs811_I2c *ccsp)
{

  switch (ccsp->state)
  {
  case CCS811_HOLD:
    // do nothing
  case CCS811_UNINIT:
    // write to APP_START (assume firmware is present)
    ccsp->i2c_trans.buf[0] = CSS811_APP_START;
    ccs811I2CTransmit(ccsp, 1U, 0U);
    ccsp->state = CCS811_BOOTED;
    break;
  case CCS811_BOOTED:
    // see event loop
    break;
  case CCS811_MEAS_MODE_OK:
    // see event loop
    break;
  case CCS811_READ_DATA:
    // see event loop
    break;
  case CCS811_WAIT:
    // when 1s has passed, request data and goto READ_DATA
     ccsp->i2c_trans.buf[0] = CSS811_ALG_RESULT_DATA;
      ccs811I2CTransmit(ccsp, 1U, sizeof(RxData_t));
      ccsp->state = CCS811_READ_DATA;
    break;
  
  default:
    break;
  }

}

void ccs811_i2c_event(Ccs811_I2c *ccsp)
{

  RxData_t rxData = {0};

  switch (ccsp->state)
  {
  case CCS811_HOLD:
    // do nothing
  case CCS811_UNINIT:
    // see periodic. (write to APP_START)
    break;
  case CCS811_BOOTED:
    // write mode to MEAS_MODE
    if(ccsp->i2c_trans.status == I2CTransSuccess) {
      ccsp->i2c_trans.status = I2CTransDone;
      ccsp->i2c_trans.buf[0] = CSS811_MEAS_MODE;
      ccsp->i2c_trans.buf[1] = CSS811_MEASMODE_EVERY_SECOND;
      // enable interrupt generation
      //ccsp->i2c_trans.buf[1] |= CSS811_MEASMODE_INT_DATARDY;
      ccs811I2CTransmit(ccsp, 2U,  0U);
      ccsp->state = CCS811_MEAS_MODE_OK;
    } else if(ccsp->i2c_trans.status == I2CTransFailed) {
      ccsp->state = CCS811_ERROR;
    }
    break;
  case CCS811_MEAS_MODE_OK:
    // request data
    if(ccsp->i2c_trans.status == I2CTransSuccess) {
      ccsp->i2c_trans.status = I2CTransDone;
      ccsp->i2c_trans.buf[0] = CSS811_ALG_RESULT_DATA;
      ccs811I2CTransmit(ccsp, 1U, sizeof(RxData_t));
      ccsp->state = CCS811_READ_DATA;
    }
    else if(ccsp->i2c_trans.status == I2CTransFailed) {
      ccsp->state = CCS811_ERROR;
    }
    break;
  case CCS811_READ_DATA:
    // read data. If status ok, goto WAIT, if noK, request data again (wait a bit?), and stay in that state
    if(ccsp->i2c_trans.status == I2CTransSuccess) {
      memcpy(&rxData, (uint8_t *) & ccsp->i2c_trans.buf, sizeof(RxData_t));
      ccsp->i2c_trans.status = I2CTransDone;
      ccsp->ccs811_status = rxData.status;
      if (rxData.status & CSS811_STATUS_ERROR) {
        // error. TODO must read ERROR_ID regesiter to clear error.
        // for now just trap state to ERROR.
        ccsp->state = CCS811_ERROR;
      } else if (rxData.status & CSS811_STATUS_DATA_READY) {
        // data ready
        ccsp->co2 = __builtin_bswap16(rxData.eCo2);
        ccsp->tvoc = __builtin_bswap16(rxData.eTvoc);
        ccsp->raw.current = rxData.rawData[0] >> 2;
        ccsp->raw.adc = ((rxData.rawData[0] & 0b11) << 8) | rxData.rawData[1];
        ccsp->resistance = 1e6f *  1.65f * ccsp->raw.adc / (1023.0f * ccsp->raw.current); // see AN373
        ccsp->data_available = true;
        ccsp->state = CCS811_WAIT;
      } else {
        // data not ready yet !
        // TODO wait a bit and retry read ?
        ccsp->state = CCS811_WAIT;
      }
    }
    else if(ccsp->i2c_trans.status == I2CTransFailed) {
      ccsp->state = CCS811_ERROR;
    }
    break;
  case CCS811_WAIT:
    // see periodic. (request data and goto READ_DATA)
    break;
  
  default:
    break;
  }
}

static bool ccs811I2CTransmit(Ccs811_I2c *ccsp, size_t txsize, size_t rxsize)
{
  return i2c_transceive(ccsp->i2c_p, &ccsp->i2c_trans, ccsp->i2c_trans.slave_addr, txsize, rxsize);
}
