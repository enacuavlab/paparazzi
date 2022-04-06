/*
 * Copyright (C) 2022 Fabien-B <fabien.bonneval [at] gmail [dot] com>
 *
 * This file is part of paparazzi
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

/** @file "modules/sensors/thermal_mlx90640.c"
 * @author Fabien-B <fabien.bonneval [at] gmail [dot] com>
 * Module for the MLX90640 themal IR sensor from Melexis.
 */

#include "modules/sensors/thermal_mlx90640.h"
#include "peripherals/mlx90640.h"

#include "core/shell.h"
#include "core/shell_arch.h"


#ifndef MLX90640_I2C_SLAVE_ADDR
#error MLX90640_I2C_SLAVE_ADDR must be defined!
#endif

struct Mlx90640 mlx;

struct i2c_periph* i2c_dev = &MLX90640_I2C_DEV;



shell_stream_t* _stream = NULL;

void stream_stealer(shell_stream_t *stream, int argc, const char * const argv[]) {
  (void)argc;
  (void)argv;
  if(argc == 0) {
    _stream = stream;
  } else {
    _stream = NULL;
  }
}



void thermal_mlx90640_init(void)
{
  mlx90640_init(&mlx, i2c_dev, MLX90640_I2C_SLAVE_ADDR);
  shell_add_entry("steal", stream_stealer);

}

void thermal_mlx90640_periodic(void)
{
  if(_stream != NULL && mlx.data_available) {
    chprintf(_stream, "%d:\r\n", mlx.subpage);
    for(size_t i=0; i<24; i+=1) {
      for(size_t j=0; j<32; j+=1) {
        chprintf(_stream, "%d ", mlx.frame[i*32+j]);
      }
      chprintf(_stream, "\r\n");
    }
    chprintf(_stream, "\r\n");
    mlx.data_available = false;
  }
  

  mlx90640_periodic(&mlx);
}

void thermal_mlx90640_event(void)
{
  mlx90640_event(&mlx);
}


