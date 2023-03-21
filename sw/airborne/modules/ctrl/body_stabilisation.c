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
#include "modules/sensors/encoder_amt22.h"
#include "modules/imu/rotate_imu.h"

#include "modules/datalink/downlink.h"

/** Enable by default */
#ifndef BODY_STAB_ENABLED
#define BODY_STAB_ENABLED TRUE
#endif

/** Default kp */
#ifndef BODY_STAB_KP
#define BODY_STAB_KP 7000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KP)

/** Default ki old 1976*/
#ifndef BODY_STAB_KI
#define BODY_STAB_KI 0.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KD)

/** Default kd */
#ifndef BODY_STAB_KD
#define BODY_STAB_KD 1565.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KD)

/** Default u_ep */
#ifndef BODY_STAB_UEQ
#define BODY_STAB_UEQ 3900
#endif
PRINT_CONFIG_VAR(BODY_STAB_UEQ)

struct BodyStab body_stab;
struct FloatEulers euler_fus;
struct FloatQuat quat_wing2fus;
struct FloatQuat quat_fus;
float motor_cmd;
float angle_wing2fus;

void body_stabilisation_init(void)
{
  body_stab.enabled = BODY_STAB_ENABLED;
  body_stab.kp = BODY_STAB_KP;
  body_stab.ki = BODY_STAB_KI;
  body_stab.kd = BODY_STAB_KD;
  body_stab.u_eq = BODY_STAB_UEQ;
  body_stab.state_integrator = 0;
  motor_cmd = 0;
}

void body_stabilisation_periodic(void)
{
  angle_wing2fus = encoder_amt22.H_g_filter.hatx[0];
  QUAT_ASSIGN(quat_wing2fus, cos(angle_wing2fus/2), 0.0, sin(angle_wing2fus/2), 0.0);

  float_quat_comp(&quat_fus, stateGetNedToBodyQuat_f(), &quat_wing2fus);
  
  float_eulers_of_quat(&euler_fus, &quat_fus);
  if(autopilot.motors_on){
    motor_cmd = body_stab.u_eq + body_stab.kp*euler_fus.theta + body_stab.ki*body_stab.state_integrator + body_stab.kd * vect_fuselage_rate.y;  //check sign 
    actuators_pprz[7] = TRIM_UPPRZ(motor_cmd);
    body_stab.state_integrator += euler_fus.theta*1/PERIODIC_FREQUENCY;
  }
 
  
}


void body_stabilisation_reset(float enabled){
  body_stab.enabled = enabled;
  body_stab.state_integrator = 0;
}

void body_stabilisation_update_kp(float kp){
  body_stab.kp = kp;
}

void body_stabilisation_update_ki(float ki){
  body_stab.ki = ki;
}

void body_stabilisation_update_kd(float kd){
  body_stab.kd = kd;
}

void body_stabilisation_update_ueq(int ueq){
  body_stab.u_eq = ueq;
}

extern void body_stabilisation_report(void){
  float f[4] = { motor_cmd, actuators_pprz[7], DegOfRad(-euler_fus.theta), angle_wing2fus};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, f); 

  /*float f[16] = {stateGetNedToBodyQuat_f()->qi, stateGetNedToBodyQuat_f()->qx, stateGetNedToBodyQuat_f()->qy, stateGetNedToBodyQuat_f()->qz,
  angle_wing2fus, quat_wing2fus.qi, quat_wing2fus.qx, quat_wing2fus.qy, quat_wing2fus.qz,
  quat_fus.qi, quat_fus.qx, quat_fus.qy, quat_fus.qz,
  DegOfRad(euler_fus.phi), DegOfRad(euler_fus.theta), DegOfRad(euler_fus.psi)};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 16, f); 
  */
}
