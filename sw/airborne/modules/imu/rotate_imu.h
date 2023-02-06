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

/** @file "modules/imu/rotate_imu.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 */

#ifndef ROTATE_IMU_H
#define ROTATE_IMU_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "filters/low_pass_filter.h"

struct RotateImu {
  bool enabled;
  float angular_speed;
  struct FloatRMat Rot_mat_f;
  struct FloatRMat Rot_frenet2imu_f;
  struct FloatVect3 centre_rot_2_imu;
  Butterworth2LowPass rate[3];
  float rate_d[3];
};

extern struct RotateImu rotate_imu;

extern void rotate_imu_init(void);
extern void rotate_imu_reset(float enabled);

/**
 * settings handlers
 */
extern void rotate_imu_update_dcm_matrix(void);

#endif  // ROTATE_IMU_H
