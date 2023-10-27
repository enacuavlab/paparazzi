/*
<<<<<<<< HEAD:sw/airborne/modules/sensors/generic_uart.h
 * Copyright (C) 2020 Freek van Tienen <freek.v.tienen@gmail.com>
========
 * Copyright (C) 2023 Fabien-B <fabien.bonneval@enac.fr>
>>>>>>>> refs/remotes/origin/panache_mfeurgard:sw/airborne/modules/sensors/panache_sensor.h
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

<<<<<<<< HEAD:sw/airborne/modules/sensors/generic_uart.h
/**
 * @file "modules/sensors/generic_uart.c"
 * @author F. van Tienen
 * Generic UART sensor, forwarding towards the GCS through telemetry
 */

#ifndef GENERIC_UART_H
#define GENERIC_UART_H

#include "std.h"

extern void generic_uart_event(void);

#endif
========
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
>>>>>>>> refs/remotes/origin/panache_mfeurgard:sw/airborne/modules/sensors/panache_sensor.h
