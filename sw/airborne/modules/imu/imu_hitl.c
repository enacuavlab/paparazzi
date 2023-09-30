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

#include "modules/imu/imu_hitl.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"
#include "modules/datalink/datalink.h"
#include "nps_sensors_params_common.h"

struct ImuHitl imu_hitl;

void imu_hitl_init(void)
{

  imu_hitl.gyro_available = false;
  imu_hitl.mag_available = false;
  imu_hitl.accel_available = false;

  // Set the default scaling
  const struct Int32Rates gyro_scale[2] = {
    {NPS_GYRO_SENSITIVITY_NUM, NPS_GYRO_SENSITIVITY_NUM, NPS_GYRO_SENSITIVITY_NUM},
    {NPS_GYRO_SENSITIVITY_DEN, NPS_GYRO_SENSITIVITY_DEN, NPS_GYRO_SENSITIVITY_DEN}
  };
  const struct Int32Rates gyro_neutral = {
    NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R
  };
  const struct Int32Vect3 accel_scale[2] = {
    {NPS_ACCEL_SENSITIVITY_NUM, NPS_ACCEL_SENSITIVITY_NUM, NPS_ACCEL_SENSITIVITY_NUM},
    {NPS_ACCEL_SENSITIVITY_DEN, NPS_ACCEL_SENSITIVITY_DEN, NPS_ACCEL_SENSITIVITY_DEN}
  };
  const struct Int32Vect3 accel_neutral = {
    NPS_ACCEL_NEUTRAL_X, NPS_ACCEL_NEUTRAL_Y, NPS_ACCEL_NEUTRAL_Z
  };
  const struct Int32Vect3 mag_scale[2] = {
    {NPS_MAG_SENSITIVITY_NUM, NPS_MAG_SENSITIVITY_NUM, NPS_MAG_SENSITIVITY_NUM},
    {NPS_MAG_SENSITIVITY_DEN, NPS_MAG_SENSITIVITY_DEN, NPS_MAG_SENSITIVITY_DEN}
  };
  const struct Int32Vect3 mag_neutral = {
    NPS_MAG_NEUTRAL_X, NPS_MAG_NEUTRAL_Y, NPS_MAG_NEUTRAL_Z
  };
  imu_set_defaults_gyro(IMU_NPS_ID, NULL, &gyro_neutral, gyro_scale);
  imu_set_defaults_accel(IMU_NPS_ID, NULL, &accel_neutral, accel_scale);
  imu_set_defaults_mag(IMU_NPS_ID, NULL, &mag_neutral, mag_scale);
}

void imu_hitl_parse_HITL_IMU(uint8_t *buf)
{
  if (DL_HITL_IMU_ac_id(buf) != AC_ID) {
    return;
  }

  RATES_ASSIGN(imu_hitl.gyro,
      NPS_GYRO_SIGN_P * DL_HITL_IMU_gp(buf),
      NPS_GYRO_SIGN_Q * DL_HITL_IMU_gq(buf),
      NPS_GYRO_SIGN_R * DL_HITL_IMU_gr(buf));
  VECT3_ASSIGN(imu_hitl.accel,
      NPS_ACCEL_SIGN_X * DL_HITL_IMU_ax(buf),
      NPS_ACCEL_SIGN_Y * DL_HITL_IMU_ay(buf),
      NPS_ACCEL_SIGN_Z * DL_HITL_IMU_az(buf));
  VECT3_ASSIGN(imu_hitl.mag,
      NPS_MAG_SIGN_X * DL_HITL_IMU_mx(buf),
      NPS_MAG_SIGN_Y * DL_HITL_IMU_my(buf),
      NPS_MAG_SIGN_Z * DL_HITL_IMU_mz(buf));

  imu_hitl.accel_available = true;
  imu_hitl.gyro_available = true;
  imu_hitl.mag_available = true;
}

void imu_hitl_event(void)
{
  uint32_t now_ts = get_sys_time_usec();
  if (imu_hitl.gyro_available) {
    AbiSendMsgIMU_GYRO_RAW(IMU_NPS_ID, now_ts, &imu_hitl.gyro, 1, NPS_PROPAGATE, NAN);
    imu_hitl.gyro_available = false;
  }
  if (imu_hitl.accel_available) {
    AbiSendMsgIMU_ACCEL_RAW(IMU_NPS_ID, now_ts, &imu_hitl.accel, 1, NPS_PROPAGATE, NAN);
    imu_hitl.accel_available = false;
  }
  if (imu_hitl.mag_available) {
    AbiSendMsgIMU_MAG_RAW(IMU_NPS_ID, now_ts, &imu_hitl.mag);
    imu_hitl.mag_available = false;
  }
}

void imu_feed_gyro_accel(void) {}
void imu_feed_mag(void) {}
