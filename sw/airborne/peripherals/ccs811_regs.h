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
 * @file peripherals/ccs811_regs.h
 * @brief Sensor driver for CCS811 sensor via I2C
 *
 */


#ifndef CCS811_REGS_H
#define CCS811_REGS_H

typedef enum  {CSS811_OK, CSS811_I2C_ERROR,
  CSS811_HEATER_SUPPLY, CSS811_HEATER_FAULT,
  CSS811_MAX_RESISTANCE,  CSS811_MEASMODE_INVALID,
  CSS811_READ_REG_INVALID,  CSS811_WRITE_REG_INVALID,
  CSS811_RESET_FAILED, CSS811_ERASE_FAILED,
  CSS811_FWLOAD_FAILED
} Css811Status;

typedef enum  {CSS811_FIRMWARE_RESET, CSS811_FIRMWARE_ERASE,
  CSS811_FIRMWARE_LOAD, CSS811_FIRMWARE_VERIFY
} Css811CurrentOperation;

typedef enum __attribute__ ((__packed__)) {
  CSS811_MEASMODE_DISABLE = 0U << 4,
  CSS811_MEASMODE_EVERY_SECOND = 0b001U << 4,
  CSS811_MEASMODE_EVERY_10_SECONDS = 0b010U << 4,
  CSS811_MEASMODE_EVERY_60_SECONDS = 0b011U << 4,
  CSS811_MEASMODE_EVERY_250_MILLISECOND = 0b100U << 4,
  CSS811_MEASMODE_INT_DATARDY = 0b1 << 3
} Css811MeasMode;


typedef struct  {
  uint8_t hw_id;
  uint8_t version_major:4;
  uint8_t version_minor:4;
  uint8_t boot_major:4;
  uint8_t boot_minor:4;
  uint8_t boot_trivial;
  uint8_t app_major:4;
  uint8_t app_minor:4;
  uint8_t app_trivial;
} Css811Version;

typedef enum __attribute__ ((__packed__)) {
  CSS811_STATUS = 0x0U,			// R   1
  CSS811_MEAS_MODE = 0x1U,		// R/W 1
  CSS811_ALG_RESULT_DATA = 0x2U,	// R   8
  CSS811_RAW_DATA = 0x3U,		// R   2
  CSS811_ENV_DATA = 0x5U,		// W   4
  CSS811_THRESHOLDS = 0x10U,		// W   4
  CSS811_BASELINE = 0x11U,		// R/W 2
  CSS811_HW_ID = 0x20U,			// R   1 -> 0x81
  CSS811_HW_VERSION = 0x21U,		// R   1
  CSS811_FW_BOOT_VERSION = 0x23U,	// R   2
  CSS811_FW_APP_VERSION = 0x24U,	// R   2
  CSS811_INTERNAL_STATE = 0xA0,		// R   1
  CSS811_ERROR_ID = 0xE0,		// R   1
  CSS811_APP_ERASE = 0xF1,		// W   4, sequence is (0xE7 0xA7 0xE6 0x09)
  CSS811_APP_DATA = 0xF2,		// W   9
  CSS811_APP_VERIFY = 0xF3,		// W   -
  CSS811_APP_START = 0xF4,		// W   -
  CSS811_SW_RESET = 0xFF		// W   4, sequence is (0x11 0xE5 0x72 0x8A)
}  Css811RegAddress;

typedef enum __attribute__ ((__packed__)) {
  CSS811_STATUS_ERROR = 1U << 0,
  CSS811_STATUS_DATA_READY = 1U << 3,
  CSS811_STATUS_APP_VALID = 1U << 4,
  CSS811_STATUS_APP_VERIFY = 1U << 5, // boot mode only
  CSS811_STATUS_APP_ERASE = 1U << 6,  // boot mode only
  CSS811_STATUS_FW_MODE = 1U << 7,    // 0 : boot mode, 1 : app mode
} Css811StatusBitMask;

typedef enum __attribute__ ((__packed__)) {
  CSS811_ERROR_WRITE_REG_INVALID = 1U << 0,
  CSS811_ERROR_READ_REG_INVALID = 1U << 1,
  CSS811_ERROR_MEASMODE_INVALID = 1U << 2,
  CSS811_ERROR_MAX_RESISTANCE = 1U << 3,
  CSS811_ERROR_HEATER_FAULT = 1U << 4,
  CSS811_ERROR_HEATER_SUPPLY = 1U << 5
} Css811ErrorBitMask;


#endif
