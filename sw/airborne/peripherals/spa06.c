/*
 * Florian Sansou florian.sansou@enac.fr
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
 */

/**
 * @file peripherals/spa06.c
 * @brief Sensor driver for SPA06/SPL06 sensor
 *
 * 
 *
 */

#include "peripherals/spa06.h"

/** local function to extract raw data from i2c/spi buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct spa06_t *spa, uint8_t *data);
static void parse_calib_data(struct spa06_t *spa, uint8_t *data);
static double compensate_pressure(struct spa06_t *spa);
static double compensate_temperature(struct spa06_t *spa);
static void spa06_register_write(struct spa06_t *spa, uint8_t reg, uint8_t value);
static void spa06_register_read(struct spa06_t *spa, uint8_t reg, uint16_t size);


/**
 * @brief Initialize the spa06 sensor instance
 * 
 * @param spa The structure containing the configuration of the spa06 instance
 */
void spa06_init(struct spa06_t *spa)
{

    spa->data_available = false;
    spa->initialized = false;
    spa->status = SPL06_STATUS_UNINIT;

  /* SPI setup */
  if(spa->bus == SPA06_SPI) {
    spa->spi.trans.cpol = SPICpolIdleHigh;
    spa->spi.trans.cpha = SPICphaEdge2;
    spa->spi.trans.dss = SPIDss8bit;
    spa->spi.trans.bitorder = SPIMSBFirst;
    spa->spi.trans.cdiv = SPIDiv16;

    spa->spi.trans.select = SPISelectUnselect;
    spa->spi.trans.slave_idx = spa->spi.slave_idx;
    spa->spi.trans.output_length = 0;
    spa->spi.trans.input_length = 0;
    spa->spi.trans.before_cb = NULL;
    spa->spi.trans.after_cb = NULL;
    spa->spi.trans.input_buf = spa->spi.rx_buf;
    spa->spi.trans.output_buf = spa->spi.tx_buf;
    spa->spi.trans.status = SPITransDone;

    // in SPI read, the first byte is garbage because writing the register address
    spa->rx_buffer = &spa->spi.rx_buf[1];
    spa->tx_buffer = spa->spi.tx_buf;
    spa->rx_length = &spa->spi.trans.input_length;
  }
  /* I2C setup */
  else {

    spa->i2c.trans.slave_addr = spa->i2c.slave_addr; // slave address
    spa->i2c.trans.status = I2CTransDone;  // set initial status: Done

    spa->rx_buffer = (uint8_t *)spa->i2c.trans.buf;
    spa->tx_buffer = (uint8_t *)spa->i2c.trans.buf;
    spa->rx_length = &spa->i2c.trans.len_r;

  }
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHO_AM_I reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading 
 * 
 * @param spa The spa06 instance
 */
void spa06_periodic(struct spa06_t *spa)
{

  /* Idle */
  if((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransDone) || 
     (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransDone)) {

      switch (spa->status) {
        case SPA06_STATUS_UNINIT:
          spa->data_available = false;
          spa->initialized = false;
          spa->status = SPA06_STATUS_IDLE;
          break;

        case SPA06_STATUS_IDLE:
          /* Request WHO_AM_I */
          spa06_register_read(spa, SPL06_REG_CHIP_ID, 1);
          break;

        case SPL06_STATUS_COEFF_AVAIL:
          break;

        case SPL06_STATUS_INIT_OK:
          break;

        case SPA06_STATUS_GET_CALIB:
          // request calibration data
          spa06_register_read(spa, SPA06_CALIB_LSB_DATA_ADDR, SPA06_CALIB_DATA_LEN);
          //process in spa06_event()
          break;

        case SPA06_STATUS_CONFIGURE:
          // From datasheet, recommended config for drone usecase:
          // osrs_p = 16, osrs_t = 2
          // IIR filter = 2 (note: this one doesn't exist...)

          spa06_register_write(spa, SPA06_CTRL_MEAS_REG_ADDR, (SPA06_OVERSAMPLING_2X_T | SPA06_OVERSAMPLING_16X_P | SPA06_POWER_NORMAL_MODE)); 

          spa06_register_write(spa, SPA06_CONFIG_REG_ADDR, (SPA06_INACTIVITY_HALF_MS | SPA06_IIR_FILTER_COEFF_16));

          break;

        case  SPA06_STATUS_READ_STATUS_REG:
          // READ THE STATUS BYTE
          spa06_register_read(spa, SPA06_STATUS_REG_ADDR, 1);
          break;

        case  SPA06_STATUS_READ_DATA_REGS:
          // READ ALL 6 DATA REGISTERS
          spa06_register_read(spa, SPA06_DATA_START_REG_ADDR, SPA06_P_T_DATA_LEN);
          break;

        default:
          break;
      }
    }
}

/**
 * @brief Should be called in the event thread
 * - Configures the sensor and reads the responses
 * - Parse and request the sensor data 
 * 
 * @param spa The spa06 instance
 */
void spa06_event(struct spa06_t *spa)
{
  /* Successful transfer */
  if((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransSuccess) || 
     (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransSuccess)) {
      switch (spa->status) {

        case SPA06_STATUS_IDLE:
          /* WHO_AM_I */
          if(spa->rx_buffer[0] == SPA06_CHIP_ID) {
            spa->device = SPA06;
            spa->status = SPL06_STATUS_COEFF_AVAIL;
          } 
          else if (spa->rx_buffer[0] == SPL06_CHIP_ID)
          {
            spa->device = SPL06;
            spa->status = SPL06_STATUS_COEFF_AVAIL;
          }
          else {
            spa->status = SPA06_STATUS_IDLE;
          }
          break;
        
        case SPL06_STATUS_COEFF_AVAIL:
          break;

        case SPL06_STATUS_INIT_OK:
          break;

        case SPA06_STATUS_GET_CALIB:
          // compute calib
          parse_calib_data(spa, &spa->rx_buffer[0]); 
          spa->status = SPA06_STATUS_CONFIGURE;
          break;

        case SPA06_STATUS_CONFIGURE:
          // nothing else to do, start reading
          spa->status = SPA06_STATUS_READ_STATUS_REG;
          spa->initialized = true;
          break;

        case SPA06_STATUS_READ_STATUS_REG:
          // check status byte
          if ((spa->rx_buffer[0] & (SPA06_EOC_BIT | SPA06_NVRAM_COPY_BIT)) == 0) {
            spa->status = SPA06_STATUS_READ_DATA_REGS;
          }
          break;

        case SPA06_STATUS_READ_DATA_REGS:
          // parse sensor data, compensate temperature first, then pressure
          parse_sensor_data(spa, &spa->rx_buffer[0]);
          compensate_temperature(spa);
          compensate_pressure(spa);
          spa->data_available = true;
          spa->status = SPA06_STATUS_READ_STATUS_REG;
          break;

        default:
          spa->status = SPA06_STATUS_GET_CALIB; // just to avoid the compiler's warning message
          break;
      }
      if(spa->bus == SPA06_I2C){
        spa->i2c.trans.status = I2CTransDone; 
      }
      else{
        spa->spi.trans.status = SPITransDone;
      }

    } else if ((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransFailed) || 
               (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransFailed)) {
      /* try again */
      if (!spa->initialized) {
        spa->status = SPA06_STATUS_UNINIT;
      }
      if(spa->bus == SPA06_I2C){
        spa->i2c.trans.status = I2CTransDone; 
      }
      else{
        spa->spi.trans.status = SPITransDone;
      }
    }

  return;
}

static void parse_sensor_data(struct spa06_t *spa, uint8_t *data)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  // SPA06 HAS THE 6 DATA REGISTERS START AT F7 AND GOING UP TO FC MSB FIRST THEN LSB AND LAST THE XLSB BYTE.
  // THE FIRST THREE BYTES ARE THE PRESSURE AND THE NEXT 3 THE TEMPERATURE.
  /* Store the parsed register values for pressure data */
  data_msb = (uint32_t)data[0] << 16;
  data_lsb = (uint32_t)data[1] << 8;
  data_xlsb = (uint32_t)data[2];
  spa->raw_pressure = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);

  /* Store the parsed register values for temperature data */
  data_msb = (uint32_t)data[3] << 16;
  data_lsb = (uint32_t)data[4] << 8;
  data_xlsb = (uint32_t)data[5];
  spa->raw_temperature = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);
}


/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure (float version)
 */
static void parse_calib_data(struct spa06_t *spa, uint8_t *data)
{
  spa->calib.dig_t1 = SPA06_CONCAT_BYTES(data[1], data[0]);
  spa->calib.dig_t2 = (int16_t)SPA06_CONCAT_BYTES(data[3], data[2]);
  spa->calib.dig_t3 = (int16_t)SPA06_CONCAT_BYTES(data[5], data[4]);

  spa->calib.dig_p1 = SPA06_CONCAT_BYTES(data[7], data[6]);
  spa->calib.dig_p2 = (int16_t)SPA06_CONCAT_BYTES(data[9], data[8]);
  spa->calib.dig_p3 = (int16_t)SPA06_CONCAT_BYTES(data[11], data[10]);
  spa->calib.dig_p4 = (int16_t)SPA06_CONCAT_BYTES(data[13], data[12]);
  spa->calib.dig_p5 = (int16_t)SPA06_CONCAT_BYTES(data[15], data[14]);
  spa->calib.dig_p6 = (int16_t)SPA06_CONCAT_BYTES(data[17], data[16]);
  spa->calib.dig_p7 = (int16_t)SPA06_CONCAT_BYTES(data[19], data[18]);
  spa->calib.dig_p8 = (int16_t)SPA06_CONCAT_BYTES(data[21], data[20]);
  spa->calib.dig_p9 = (int16_t)SPA06_CONCAT_BYTES(data[23], data[22]);

  return;
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in float data type.
 */
static double compensate_temperature(struct spa06_t *spa)
{
  double var1 = (((double)spa->raw_temperature / 16384.0) - ((double)spa->calib.dig_t1 / 1024.0)) * ((double)spa->calib.dig_t2);
  double var2 = ((double)spa->raw_temperature / 131072.0) - ((double)spa->calib.dig_t1 / 8192.0);
  var2 = (var2 * var2) * (double)spa->calib.dig_t3;
  int64_t t_fine = (int64_t)(var1 + var2);

  /* Store t_lin in dev. structure for pressure calculation */
  spa->calib.t_fine = t_fine;
  /* Store compensated temperature in float in structure */
  spa->temperature = (((var1 + var2) / 5120.f) * 100);

  return (double)spa->temperature;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static double compensate_pressure(struct spa06_t *spa)
{
  double var1;
  double var2;
  double p;

  var1 = ((double)spa->calib.t_fine / 2) - 64000.0;
  var2 = (var1 * var1 * (double)spa->calib.dig_p5) / 32768.0;
  var2 = var2 + (var1 * (double)spa->calib.dig_p5 * 2.0);
  var2 = (var2 / 4.0) + ((double)spa->calib.dig_p4 * 65536.0);
  var1 = (((double)spa->calib.dig_p3 * var1 * (var1 / 524288.0)) + ((double)spa->calib.dig_p2 * var1)) / 524288.0;
  var1 = (1 + (var1 / 32768.0)) * (double)spa->calib.dig_p1;
  p = 1048576.0 - (double)spa->raw_pressure;
  p = (p - (var2 / 4096.0)) * (6250.0 / var1);
  var1 = ((double)spa->calib.dig_p9 * p) * (p / 2147483648.0);
  var2 = (p * ((double)spa->calib.dig_p8)) / 32768.0;
  p = p + ((var1 + var2 + (double)spa->calib.dig_p7) / 16.0);
  spa->pressure = p;

  return (p);
}


/**
 * @brief Write a register with a value
 * 
 * @param spa The spa06 instance
 * @param reg The register address
 * @param value The value to write to the register
 */
static void spa06_register_write(struct spa06_t *spa, uint8_t reg, uint8_t value) {

  spa->tx_buffer[1] = value;

  /* SPI transaction */
  if(spa->bus == SPA06_SPI) {
    spa->tx_buffer[0] = (reg & 0x7F); //write command (bit 7 = RW = '0')
    spa->spi.trans.output_length = 2;
    spa->spi.trans.input_length = 0;
    spi_submit(spa->spi.p, &(spa->spi.trans));
  }
  /* I2C transaction */
  else {
     spa->tx_buffer[0] = reg;
    i2c_transmit(spa->i2c.p, &(spa->i2c.trans), spa->i2c.slave_addr, 2);
  }
}

/**
 * @brief Read a register 
 * 
 * @param spa The spa06 instance
 * @param reg The register address
 * @param size The size to read (already 1 is added for the transmission of the register to read)
 */
static void spa06_register_read(struct spa06_t *spa, uint8_t reg, uint16_t size) {

  spa->tx_buffer[0] = reg | SPA06_READ_FLAG;
  /* SPI transaction */
  if(spa->bus ==   SPA06_SPI) {
    spa->spi.trans.output_length = 2;
    spa->spi.trans.input_length = size+1;
    spa->tx_buffer[1] = 0;
    spi_submit(spa->spi.p, &(spa->spi.trans));
  }
  /* I2C transaction */
  else {
    i2c_transceive(spa->i2c.p, &(spa->i2c.trans), spa->i2c.slave_addr, 1, size);
  }
}