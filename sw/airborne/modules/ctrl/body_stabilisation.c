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

/** @file "modules/ctrl/body_stabilisation.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Module for stabilising the fuselage of a horizontal tiltwing
 */

#include "modules/ctrl/body_stabilisation.h"
#include "modules/actuators/actuators.h"
#include "filters/high_gain_filter.h"
#include "modules/imu/rotate_imu.h"

#include "modules/datalink/downlink.h"

/** Enable by default */
#ifndef BODY_STAB_ENABLED
#define BODY_STAB_ENABLED TRUE
#endif

/** Default kp */
#ifndef BODY_STAB_KP
#define BODY_STAB_KP 5000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KP)

/** Default kd */
#ifndef BODY_STAB_KD
#define BODY_STAB_KD 0.1f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KD)

struct BodyStab body_stab;

void body_stabilisation_init(void)
{
  body_stab.enabled = BODY_STAB_ENABLED;
  body_stab.kp = BODY_STAB_KP;
  body_stab.kd = BODY_STAB_KD;
}

void body_stabilisation_periodic(void)
{
  float angle_wing2fus = -H_g_filter_rot.hatx[0];
  struct FloatQuat quat_wing2fus = {cos(angle_wing2fus/2), 0, sin(angle_wing2fus/2),0};

  struct FloatQuat quat_fus;
  float_quat_comp(&quat_fus, stateGetNedToBodyQuat_f(), &quat_wing2fus);
  struct FloatEulers euler_fus;
  float_eulers_of_quat(&euler_fus, &quat_fus);

  float motor_cmd = body_stab.kp*euler_fus.theta + body_stab.kd * vect_fuselage_rate.y; 
  actuators_pprz[7] = TRIM_UPPRZ(motor_cmd);
  
  //float f[3] = { motor_cmd, actuators_pprz[7], DegOfRad(euler_fus.theta)};
  //DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3, f); 
}


void body_stabilisation_reset(float enabled){
  body_stab.enabled = enabled;
}

void body_stabilisation_update_kp(float kp){
  body_stab.kp = kp;
}

void body_stabilisation_update_kd(float kd){
  body_stab.kd = kd;
}

