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
#include "state.h"

#include "modules/datalink/downlink.h"

/** Enable by default */
#ifndef BODY_STAB_ENABLED
#define BODY_STAB_ENABLED TRUE
#endif

/** Default kp motor */
#ifndef BODY_STAB_KP_M
#define BODY_STAB_KP_M 7000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KP_M)

/** Default ki motor old 1976*/
#ifndef BODY_STAB_KI_M
#define BODY_STAB_KI_M 0.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KI_M)

/** Default kd motor*/
#ifndef BODY_STAB_KD_M
#define BODY_STAB_KD_M 3000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KD_M)

/** Default u_ep motor*/
#ifndef BODY_STAB_UEQ_M
#define BODY_STAB_UEQ_M 3900
#endif
PRINT_CONFIG_VAR(BODY_STAB_UEQ_M)

/** Default kp elevator*/
#ifndef BODY_STAB_KP_E
#define BODY_STAB_KP_E 10000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KP_E)

/** Default ki elevator */
#ifndef BODY_STAB_KI_E
#define BODY_STAB_KI_E 0.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KI_E)

/** Default kd elevator*/
#ifndef BODY_STAB_KD_E
#define BODY_STAB_KD_E 3000.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_KD_E)

/** Max value of integral state  */
#ifndef BODY_STAB_CLAMP_INT
#define BODY_STAB_CLAMP_INT 5.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_CLAMP_INT)

/** Lower bound to switch motor elevator  */
#ifndef BODY_STAB_AIR_LB
#define BODY_STAB_AIR_LB 8.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_AIR_LB)

/** Upper bound to switch motor elevator  */
#ifndef BODY_STAB_AIR_UB
#define BODY_STAB_AIR_UB 12.f
#endif
PRINT_CONFIG_VAR(BODY_STAB_AIR_UB)

struct BodyStab body_stab;
struct FloatEulers euler_fus;
struct FloatQuat quat_wing2fus;
struct FloatQuat quat_fus;
float motor_cmd;
float elevator_cmd;
float angle_wing2fus;

void body_stabilisation_init(void)
{
  body_stab.enabled = BODY_STAB_ENABLED;
  body_stab.kp_m = BODY_STAB_KP_M;
  body_stab.ki_m = BODY_STAB_KI_M;
  body_stab.kd_m = BODY_STAB_KD_M;
  body_stab.u_eq_m = BODY_STAB_UEQ_M;
  body_stab.kp_e = BODY_STAB_KP_E;
  body_stab.ki_e = BODY_STAB_KI_E;
  body_stab.kd_e = BODY_STAB_KD_E;
  body_stab.clamp_integrator = BODY_STAB_CLAMP_INT;
  body_stab.airspeed_lb = BODY_STAB_AIR_LB;
  body_stab.airspeed_ub = BODY_STAB_AIR_UB;
  body_stab.state_integrator = 0;
  body_stab.discrete_state = 0;
  motor_cmd = 0;
  elevator_cmd = 0;
}

void body_stabilisation_periodic(void)
{
  float airspeed = stateGetAirspeed_f();
  angle_wing2fus = encoder_amt22.H_g_filter.hatx[0];
  QUAT_ASSIGN(quat_wing2fus, cos(angle_wing2fus/2), 0.0, sin(angle_wing2fus/2), 0.0);

  float_quat_comp(&quat_fus, stateGetNedToBodyQuat_f(), &quat_wing2fus);
  
  float_eulers_of_quat(&euler_fus, &quat_fus);
  if(autopilot.motors_on){
    //hysteresis switch mechanism
    if ((airspeed > body_stab.airspeed_ub && body_stab.discrete_state == 0) ||
        (airspeed < body_stab.airspeed_lb && body_stab.discrete_state == 1))  //|| (euler_fus.theta < 0 && body_stab.discrete_state == 0)
    {
      body_stab.discrete_state = 1 - body_stab.discrete_state;
    }

    if (body_stab.discrete_state){
      // Use elevator discrete_state = 1
      /*
      float kp_without_wind = 20000; //Proportional gain at zero speed
      float speed_at_normal_kp = 8.0;
      Bound(airspeed, 0.0, speed_at_normal_kp); // Bound at max speed_at_normal_kp m/s
      float scale = 1.0 - (1.0/speed_at_normal_kp)*airspeed; // scale zero speed sclae = 0 and speed_at_normal_kp m/s scale = 0
      float kp_schedule = scale*kp_without_wind + (1-scale)*body_stab.kp_e; //linear mapping from kp_without_wind to body_stab.kp_e
      elevator_cmd = -(kp_schedule*euler_fus.theta + body_stab.ki_e*body_stab.state_integrator + body_stab.kd_e * vect_fuselage_rate.y);  
      */
      elevator_cmd = -(body_stab.kp_e*euler_fus.theta + body_stab.ki_e*body_stab.state_integrator + body_stab.kd_e * vect_fuselage_rate.y);
      actuators_pprz[6] = TRIM_PPRZ(elevator_cmd); //elevator
      actuators_pprz[7] = TRIM_UPPRZ(0); //motor
    }
    else{
      // Use motor discrete_state = 0
      
      /*
      if(euler_fus.theta >= 0){
        motor_cmd = body_stab.u_eq_m + body_stab.kp_m*euler_fus.theta + body_stab.ki_m*body_stab.state_integrator + body_stab.kd_m * vect_fuselage_rate.y;
      }
      else{
        float gain_multiply = 2.0; //gain for negative angle on fuselage
        motor_cmd = body_stab.u_eq_m + gain_multiply*body_stab.kp_m*euler_fus.theta + body_stab.ki_m*body_stab.state_integrator + body_stab.kd_m * vect_fuselage_rate.y;
      }
      */

      motor_cmd = body_stab.u_eq_m + body_stab.kp_m*euler_fus.theta + body_stab.ki_m*body_stab.state_integrator + body_stab.kd_m * vect_fuselage_rate.y;
      actuators_pprz[6] = TRIM_PPRZ(0); //elevator
      actuators_pprz[7] = TRIM_UPPRZ(motor_cmd); //motor
    }


    if (autopilot_in_flight()){
      if ((body_stab.state_integrator < body_stab.clamp_integrator && body_stab.state_integrator > -body_stab.clamp_integrator) || (body_stab.state_integrator*euler_fus.theta < 0)){
        body_stab.state_integrator += euler_fus.theta*1/PERIODIC_FREQUENCY;
      }
    }
  }
  else
  {
    elevator_cmd = 0;
    motor_cmd = 0;
    actuators_pprz[6] = TRIM_PPRZ(elevator_cmd); //elevator
    actuators_pprz[7] = TRIM_UPPRZ(motor_cmd); //motor
    body_stab.state_integrator = 0;
    body_stab.discrete_state = 0;
  }
 
  
}


void body_stabilisation_reset(float enabled){
  body_stab.enabled = enabled;
  body_stab.state_integrator = 0;
}

void body_stabilisation_update_kp_m(float kp){
  body_stab.kp_m = kp;
}

void body_stabilisation_update_ki_m(float ki){
  body_stab.ki_m = ki;
}

void body_stabilisation_update_kd_m(float kd){
  body_stab.kd_m = kd;
}

void body_stabilisation_update_ueq_m(int ueq){
  body_stab.u_eq_m = ueq;
}

void body_stabilisation_update_kp_e(float kp){
  body_stab.kp_e = kp;
}

void body_stabilisation_update_ki_e(float ki){
  body_stab.ki_e = ki;
}

void body_stabilisation_update_kd_e(float kd){
  body_stab.kd_e = kd;
}

void body_stabilisation_update_clamp_int(float clamp_val){
  body_stab.clamp_integrator = clamp_val;
}

void body_stabilisation_update_airspeed_lb(float lb){
  body_stab.airspeed_lb = lb;
}

void body_stabilisation_update_airspeed_ub(float ub){
  body_stab.airspeed_ub = ub;
}

extern void body_stabilisation_report(void){
  float f[7] = { motor_cmd, elevator_cmd, actuators_pprz[6], actuators_pprz[7], DegOfRad(euler_fus.theta), angle_wing2fus, body_stab.discrete_state};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 7, f); 

  /*float f[16] = {stateGetNedToBodyQuat_f()->qi, stateGetNedToBodyQuat_f()->qx, stateGetNedToBodyQuat_f()->qy, stateGetNedToBodyQuat_f()->qz,
  angle_wing2fus, quat_wing2fus.qi, quat_wing2fus.qx, quat_wing2fus.qy, quat_wing2fus.qz,
  quat_fus.qi, quat_fus.qx, quat_fus.qy, quat_fus.qz,
  DegOfRad(euler_fus.phi), DegOfRad(euler_fus.theta), DegOfRad(euler_fus.psi)};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 16, f); 
  */
}
