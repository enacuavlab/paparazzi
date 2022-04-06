/*
 *
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
 */

/** @file peripherals/mlx90640.c
 *  Driver for MPL3115A2 baro sensor.
 */

#include "peripherals/mlx90640.h"
#include "std.h"
#include "mcu_periph/i2c.h"

#define NB_PACKETS  (MLX90640_PIXEL_NB / DATA_CHUNK_SIZE)

#define DATA_CHUNK_SIZE (I2C_BUF_LEN/2)


void write_big_endian(volatile uint8_t* buf, uint16_t data) {
  buf[0] = data >> 8;
  buf[1] = data & 0xFF;
  
}

void mlx90640_init(struct Mlx90640 *mlx, struct i2c_periph *i2c_p, uint8_t addr)
{

  /* set i2c_peripheral */
  mlx->i2c_p = i2c_p;

  /* slave address */
  mlx->trans.slave_addr = addr;

  mlx->trans.status = I2CTransDone;

  mlx->initialized = false;
  mlx->status = MLX90640_INIT;
  mlx->status_reg = 0;
  mlx->packet_number = 0;
}

// Normal reading
void mlx90640_read(struct Mlx90640 *mlx)
{
  if(mlx->initialized && mlx->trans.status == I2CTransDone) {
    // request read status register
    write_big_endian(mlx->trans.buf, MLX90640_REG_STATUS);
    i2c_transceive(mlx->i2c_p, &mlx->trans, mlx->trans.slave_addr, 2, 2);
    mlx->status = MLX90640_POLL_DATA;
  }
}

// return true if data was requested, of false if no more data is needed
bool request_data(struct Mlx90640 *mlx) {
  _Static_assert (I2C_BUF_LEN >= DATA_CHUNK_SIZE*2, "I2C_BUF_LEN too small!");

  if(mlx->packet_number < NB_PACKETS) {
    write_big_endian(mlx->trans.buf, MLX90640_DATA_START + mlx->packet_number*DATA_CHUNK_SIZE);
    i2c_transceive(mlx->i2c_p, &mlx->trans, mlx->trans.slave_addr, 2, DATA_CHUNK_SIZE*2);
    mlx->packet_number += 1;
    return true;
  } else {
    return false;
  }  
}

void handle_data(struct Mlx90640 *mlx) {
  int16_t* data = (int16_t*)mlx->trans.buf;
  for(size_t i=0; i<DATA_CHUNK_SIZE; i+=1) {
    size_t frame_index = (mlx->packet_number-1)*DATA_CHUNK_SIZE + i;
    mlx->frame[frame_index] = __builtin_bswap16(data[i]);
  }
  if(mlx->packet_number >= NB_PACKETS) {
    mlx->data_available = true;
  }
}

void mlx90640_event(struct Mlx90640 *mlx)
{
  if (mlx->initialized) {
    if (mlx->trans.status == I2CTransFailed) {
      mlx->status = MLX90640_IDLE;
      mlx->trans.status = I2CTransDone;
    }
    else if (mlx->trans.status == I2CTransSuccess) {
      if(mlx->status ==  MLX90640_IDLE) {
        mlx->trans.status = I2CTransDone;
      }
      else if(mlx->status == MLX90640_POLL_DATA) {
        // Successfull reading and new data available
        mlx->status_reg = mlx->trans.buf[0]<<8 | mlx->trans.buf[1];
        mlx->trans.status = I2CTransDone;
        if(mlx->status_reg & MLX90640_STATUS_NEW_DATA) {
          // next data will concern this subpage
          mlx->subpage = mlx->status_reg & MLX90640_STATUS_SUBPAGE;
          // reset packet_number
          mlx->packet_number = 0;
          request_data(mlx);
          mlx->status = MLX90640_READ_DATA;
        } else {
          // no data ready, return to MLX90640_IDLE, and poll again at next periodic.
          mlx->status = MLX90640_IDLE;
        }
        
      }
      else if(mlx->status == MLX90640_READ_DATA) {
        handle_data(mlx);
        mlx->trans.status = I2CTransDone;
        if(!request_data(mlx)) {
          //no more data requested, clear the status register bit "data available"
          uint16_t stat = mlx->status_reg & ~MLX90640_STATUS_NEW_DATA;
          write_big_endian(mlx->trans.buf, MLX90640_REG_STATUS);
          write_big_endian(mlx->trans.buf+2, stat);
          i2c_transceive(mlx->i2c_p, &mlx->trans, mlx->trans.slave_addr, 4, 0);
          mlx->status = MLX90640_CLEAR_DR;
        }
      }
      else if(mlx->status == MLX90640_CLEAR_DR) {
        mlx->status = MLX90640_IDLE;
        mlx->trans.status = I2CTransDone;
      }
    }
  }
  else {
    if (mlx->trans.status == I2CTransSuccess) {
      mlx->initialized = true;
    }
  }
}

void mlx90640_periodic(struct Mlx90640 *mlx)
{
  if (!mlx->initialized && mlx->status == MLX90640_INIT) {
    write_big_endian(mlx->trans.buf, MLX90640_REG_CR);
    // 0x1901
    write_big_endian(mlx->trans.buf+2, 0x1901);
    i2c_transmit(mlx->i2c_p, &mlx->trans, mlx->trans.slave_addr, 4);
    mlx->status = MLX90640_IDLE;
  }
  if (mlx->initialized && mlx->status == MLX90640_IDLE) {
    mlx90640_read(mlx);
  }
}
