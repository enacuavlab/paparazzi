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

/** @file "modules/imu/imu_colibri.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * IMU driver for Colibri
- MPU60X0 tawaki board
- MPU9250 wing frame
 */

#ifndef IMU_COLIBRI_H
#define IMU_COLIBRI_H

extern void imu_colibri_init(void);
extern void imu_colibri_periodic(void);
extern void imu_colibri_event(void);

#endif  // IMU_COLIBRI_H
