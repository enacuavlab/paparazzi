/*
 * Copyright (C) 2025 Julia Cabarbaye <julia.cabarbaye1@gmail.com>
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

/** @file "modules/cam_control/gimbal_ctrl.c"
 * @author Julia Cabarbaye <julia.cabarbaye1@gmail.com>
 * caddx gm3 gimbal control sbus
 */

#include "modules/cam_control/gimbal_ctrl.h"
#include "modules/actuators/actuators.h"
#include "generated/airframe.h"
#include "modules/datalink/datalink.h"
#include "pprzlink/dl_protocol.h"   // datalink messages

void gimbal_ctrl_init(void)
{
  ActuatorSet(CAM_MODE, MIN_PPRZ);
  ActuatorSet(CAM_ROLL, 0);
  ActuatorSet(CAM_PITCH, 0);
  ActuatorSet(CAM_YAW, 0);
  ActuatorSet(CAM_SENS, 0);
}

void gimbal_ctrl_periodic(void)
{
  /*
  static int yaw = 0;
  static bool up = true;

  if(up && yaw >= MAX_PPRZ/4) {
    up = false;
  }
  else if(!up && yaw <= MIN_PPRZ/4) {
    up = true;
  }

  if(up) {
    yaw += 200;
  } else {
    yaw -= 200;
  }
  ActuatorSet(CAM_YAW, yaw);
  */
}

int16_t convert_to_CAM_boundary(int8_t channel, int8_t coef)
{
  return (int16_t) (((channel * coef / (127.0f * 127.0f)) + (10.0f / 127.0f)) * MAX_PPRZ);
}

void gimbal_ctrl_datalink(uint8_t* buf)
{
  //uint8_t ac_id = pprzlink_get_DL_RC_UP_ac_id(buf);  
  uint8_t lenght = pprzlink_get_RC_UP_channels_length(buf);
  int8_t* channels = pprzlink_get_DL_RC_UP_channels(buf);

  static int16_t roll = 0; 
  static int16_t pitch = 0;
  static int16_t yaw = 0;

  if (lenght < 3) return;

  roll = convert_to_CAM_boundary(channels[0], 32.0f);
  pitch = convert_to_CAM_boundary(channels[1], 50.0f);
  yaw = convert_to_CAM_boundary(channels[2], 100.0f);

  ActuatorSet(CAM_ROLL, roll);
  ActuatorSet(CAM_PITCH, pitch);
  ActuatorSet(CAM_YAW, yaw);

}


