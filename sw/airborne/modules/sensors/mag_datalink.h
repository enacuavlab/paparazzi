/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file "modules/sensors/mag_datalink.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Optitrack used as indoor magnetometer
Optitrack can determine the drone's orientation, we define a magnetic field relative to the optitrack frame of reference and express it in the body frame  to send the message IMU_MAG_RAW.
 */

#ifndef MAG_DATALINK_H
#define MAG_DATALINK_H

#include "std.h"
#include "generated/airframe.h"


extern void mag_datalink_init(void);

extern void mag_datalink_parse_EXTERNAL_POSE(uint8_t *buf);

#endif  // MAG_DATALINK_H
