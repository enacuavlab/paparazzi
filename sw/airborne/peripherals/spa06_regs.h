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
 * @file peripherals/spa06_regs.h
 * @brief Sensor driver for SPA06/SPL06 sensor register definition
 *
 * 
 *
 */

#ifndef SPA06_REGS_H
#define SPA06_REGS_H

#include "std.h"

// I2C addresses (8 bits)
#define SPA06_I2C_ADDR                         0x77
#define SPA06_I2C_ADDR_ALT                     0x76

/**\ chip identifier */
#define SPL06_CHIP_ID                          0x10
#define SPA06_CHIP_ID                          0x11

#define SPL06_READ_FLAG                        0x80

#define SPL06_REG_PRESSURE_B2                  0x00    // Pressure MSB Register
#define SPL06_REG_PRESSURE_B1                  0x01    // Pressure middle byte Register
#define SPL06_REG_PRESSURE_B0                  0x02    // Pressure LSB Register
#define SPL06_REG_PRESSURE_START               SPL06_REG_PRESSURE_B2
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_REG_TEMPERATURE_B2               0x03    // Temperature MSB Register
#define SPL06_REG_TEMPERATURE_B1               0x04    // Temperature middle byte Register
#define SPL06_REG_TEMPERATURE_B0               0x05    // Temperature LSB Register
#define SPL06_REG_TEMPERATURE_START            SPL06_REG_TEMPERATURE_B2
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_REG_PRESSURE_CFG                 0x06    // Pressure config
#define SPL06_REG_TEMPERATURE_CFG              0x07    // Temperature config
#define SPL06_REG_MODE_AND_STATUS              0x08    // Mode and status
#define SPL06_REG_INT_AND_FIFO_CFG             0x09    // Interrupt and FIFO config
#define SPL06_REG_INT_STATUS                   0x0A    // Interrupt and FIFO config
#define SPL06_REG_FIFO_STATUS                  0x0B    // Interrupt and FIFO config
#define SPL06_REG_RST                          0x0C    // Softreset Register
#define SPL06_REG_CHIP_ID                      0x0D    // Chip ID Register
#define SPL06_REG_CALIB_COEFFS_START           0x10
#define SPL06_REG_CALIB_COEFFS_END             0x21
#define SPA06_REG_CALIB_COEFFS_END             0x24

// PRESSURE_CFG_REG
#define SPL06_PRES_RATE_32HZ				           (0x05 << 4)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)
#define SPL06_TEMP_RATE_32HZ				           (0x05 << 4)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature
#define SPL06_MEAS_CON_PRE_TEM				         0x07

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8

// Don't set oversampling higher than 8 or the measurement time will be higher than 20ms (timer period)
#define SPL06_PRESSURE_OVERSAMPLING            8
#define SPL06_TEMPERATURE_OVERSAMPLING         8


#define SPL06_BACKGROUND_SAMPLE_RATE	         32


#endif /* SPA06_REGS_H */
