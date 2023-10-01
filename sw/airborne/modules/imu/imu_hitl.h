/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

#ifndef IMU_HITL_H
#define IMU_HITL_H

#include "modules/imu/imu.h"

#include "generated/airframe.h"

struct ImuHitl {
  uint8_t mag_available;
  uint8_t accel_available;
  uint8_t gyro_available;

  struct Int32Rates gyro;
  struct Int32Vect3 accel;
  struct Int32Vect3 mag;
};

extern struct ImuHitl imu_hitl;

extern void imu_hitl_init(void);
extern void imu_hitl_event(void);
extern void imu_hitl_parse_HITL_IMU(uint8_t *buf);
extern void imu_hitl_parse_HITL_AIR_DATA(uint8_t *buf);

// dummy
extern void imu_feed_gyro_accel(void);
extern void imu_feed_mag(void);

#endif /* IMU_HITL_H */

