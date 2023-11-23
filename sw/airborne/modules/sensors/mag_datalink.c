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

/** @file "modules/sensors/mag_datalink.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Optitrack used as indoor magnetometer
Optitrack can determine the drone's orientation, we define a magnetic field relative to the optitrack frame of reference and express it in the body frame  to send the message IMU_MAG_RAW.
 */

#include "modules/sensors/mag_datalink.h"

#include "modules/core/abi.h"
#include "modules/imu/imu.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"

struct FloatVect3 mag_field = {1., 0., 0.};

void mag_datalink_init(void)
{
  
}

/** Parse the full EXTERNAL_POSE message and publish as mag through ABI */
void mag_datalink_parse_EXTERNAL_POSE(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  uint32_t tow = DL_EXTERNAL_POSE_timestamp(buf);

  struct FloatQuat body_q;
  body_q.qi = DL_EXTERNAL_POSE_body_qi(buf);
  body_q.qx = DL_EXTERNAL_POSE_body_qy(buf);
  body_q.qy = DL_EXTERNAL_POSE_body_qx(buf);
  body_q.qz = DL_EXTERNAL_POSE_body_qz(buf);

  struct FloatVect3 mag_float;

  float_quat_vmult(&mag_float, &body_q, &mag_field);

  struct Int32Vect3 mag = {MAG_BFP_OF_REAL(mag_float.x),
                           MAG_BFP_OF_REAL(mag_float.y),
                           MAG_BFP_OF_REAL(mag_float.z)}

  uint32_t now_ts = get_sys_time_usec();

  // Publish mag data
  AbiSendMsgIMU_MAG_RAW(MAG_DATALINK_SENDER_ID, now_ts, &mag);
}



