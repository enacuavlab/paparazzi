/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file peripherals/mlx90640.h
 * Driver for the IR array MLX90640 from Melexis (i2c)
 */

#ifndef MLX90640_H
#define MLX90640_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Device address (8 bits) */
#define MLX90640_I2C_ADDR 0xC0



#define MLX90640_ROM_LOW 0x0000
#define MLX90640_ROM_HIGH 0x03FF
#define MLX90640_RAM_LOW 0x0400
#define MLX90640_RAM_HIGH 0x07FF
#define MLX90640_EEPROM_LOW 0x2400
#define MLX90640_EEPROM_HIGH 0x273F
#define MLX90640_REGISTERS_LOW 0x800D
#define MLX90640_REGISTERS_HIGH 0x8010

/* Registers */
#define MLX90640_REG_STATUS 0x8000
#define MLX90640_REG_CR     0x800D

#define MLX90640_DATA_START 0x0400
#define MLX90640_DATA_END   0x06FF


#define MLX90640_PIXEL_NB 768

typedef enum __attribute__ ((__packed__)) {
  // last measured subpage
  // 000 : subpage 0
  // 001 : subpage 1
  // other : reserved
  MLX90640_STATUS_SUBPAGE = 0b111U << 0, 
  // new data vailable in RAM
  // reset this bit once data has been read
  MLX90640_STATUS_NEW_DATA = 1U << 3,
} Mlx90640StatusBitMask;

typedef enum __attribute__ ((__packed__)) {
  MLX90640_SUBPAGE_0 = 0b000 << 0,
  MLX90640_SUBPAGE_1 = 0b001 << 0,
} Mlx90640Subpage;

enum Mlx90640Status {
  MLX90640_INIT,
  MLX90640_IDLE,
  MLX90640_POLL_DATA,   // read "data ready" bit
  MLX90640_READ_DATA,   // read data (may take multiple requests)
  MLX90640_CLEAR_DR,    // clear data ready bit
};

struct Mlx90640 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction trans;       ///< I2C transaction for reading and configuring
  enum Mlx90640Status status;
  Mlx90640Subpage subpage;          ///< next subpage to be read
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  uint16_t status_reg;
  int packet_number;

  int16_t frame[MLX90640_PIXEL_NB];

};

// Functions
extern void mlx90640_init(struct Mlx90640 *mlx, struct i2c_periph *i2c_p, uint8_t addr);
extern void mlx90640_read(struct Mlx90640 *mlx);
extern void mlx90640_event(struct Mlx90640 *mlx);
extern void mlx90640_periodic(struct Mlx90640 *mlx);

#endif // MPL3115_H
