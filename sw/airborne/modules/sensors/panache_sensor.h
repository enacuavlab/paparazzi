/*
 * Copyright (C) 2023 Fabien-B <fabien.bonneval@enac.fr>
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

/** @file "modules/sensors/panache_sensor.h"
 * @author Fabien-B <fabien.bonneval@enac.fr>
 * Get data from Panache sensors, or datalink (for simulation)
 */

#ifndef PANACHE_SENSOR_H
#define PANACHE_SENSOR_H

#include "stdint.h"

extern void panache_init(void);
extern void panache_dl_cb(uint8_t* buf);	// PAYLOAD_COMMAND

#endif  // PANACHE_SENSOR_H
