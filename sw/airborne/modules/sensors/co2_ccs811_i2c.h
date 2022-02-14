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
 * @file modules/sensors/co2_ccs811_i2c.h
 * ScioSense CCS811 CO2 I2C sensor interface.
 *
 * This reads the values for CO2 from the ScioSense CCS811 sensor through I2C.
 */

#ifndef CO2_CCS811_I2C_H
#define CO2_CCS811_I2C_H

#include "peripherals/ccs811_i2c.h"

extern Ccs811_I2c co2_ccs811;

extern void co2_ccs811_init(void);
extern void co2_ccs811_periodic(void);
extern void co2_ccs811_event(void);

#endif
