/*
 * Copyright (C) Jacson Barth <jacsonm2@gmail.com>
 * ENAC UAV Lab
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_mfc.c
 * @brief ENAC UAV Lab
 * This control algorithm is Model-Free Control (MFC)
 *
 * This is an implementation of the publication in the
 * International American Control Conference : Full Model-Free 
 * Control Architecture for Hybrid UAVs
 */

#include "state.h"
#include "std.h"
#include "generated/airframe.h"
//#include "navigation.h"
#include "subsystems/ins/ins_int.h"
#include "paparazzi.h"
#include "autopilot.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/radio_control.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/actuators.h"
#include <stdio.h>

#include "firmwares/rotorcraft/stabilization/stabilization_mfc.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "subsystems/datalink/telemetry.h"

// Debugging ...
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifndef STABILIZATION_MFC_CONTROL_FREQUENCY
#define STABILIZATION_MFC_CONTROL_FREQUENCY 512
#endif

/* Parameters for roll MFC */
#ifndef STABILIZATION_MFC_ROLL_ID
#define STABILIZATION_MFC_ROLL_ID 0
#endif

#ifndef STABILIZATION_MFC_ROLL_TIME_TRAJECTORY
#define STABILIZATION_MFC_ROLL_TIME_TRAJECTORY 70.f
#endif

#ifndef STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP 1.f
#endif

#ifndef STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW
#define STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW 1.f
#endif

#ifndef STABILIZATION_MFC_ROLL_ALPHA
#define STABILIZATION_MFC_ROLL_ALPHA 110.f
#endif

#ifndef STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN 2.f
#endif

#ifndef STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN
#define STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN 0.1f
#endif

#ifndef STABILIZATION_MFC_ROLL_COMMAND_FILTER
#define STABILIZATION_MFC_ROLL_COMMAND_FILTER 1.f
#endif

/* Parameters for pitch MFC */
#ifndef STABILIZATION_MFC_PITCH_ID
#define STABILIZATION_MFC_PITCH_ID 1
#endif

#ifndef STABILIZATION_MFC_PITCH_TIME_TRAJECTORY
#define STABILIZATION_MFC_PITCH_TIME_TRAJECTORY 90.0f
#endif

#ifndef STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP 1.
#endif

#ifndef STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW
#define STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW 1.f
#endif

#ifndef STABILIZATION_MFC_PITCH_ALPHA
#define STABILIZATION_MFC_PITCH_ALPHA 650.f
#endif

#ifndef STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN 4.f
#endif

#ifndef STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN
#define STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN 0.1f
#endif

#ifndef STABILIZATION_MFC_PITCH_COMMAND_FILTER
#define STABILIZATION_MFC_PITCH_COMMAND_FILTER 1.f
#endif

/* Parameters for yaw MFC */
#ifndef STABILIZATION_MFC_YAW_ID
#define STABILIZATION_MFC_YAW_ID 2
#endif

#ifndef STABILIZATION_MFC_YAW_TIME_TRAJECTORY
#define STABILIZATION_MFC_YAW_TIME_TRAJECTORY 80.f
#endif

#ifndef STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP 1.f
#endif

#ifndef STABILIZATION_MFC_YAW_INTEGRATION_WINDOW
#define STABILIZATION_MFC_YAW_INTEGRATION_WINDOW 1.f
#endif

#ifndef STABILIZATION_MFC_YAW_ALPHA
#define STABILIZATION_MFC_YAW_ALPHA 280.f
#endif

#ifndef STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN 2.5f
#endif

#ifndef STABILIZATION_MFC_YAW_DERIVATIVE_GAIN
#define STABILIZATION_MFC_YAW_DERIVATIVE_GAIN 0.1f
#endif

#ifndef STABILIZATION_MFC_YAW_COMMAND_FILTER
#define STABILIZATION_MFC_YAW_COMMAND_FILTER 1.f
#endif

#ifndef STABILIZATION_MFC_MAX_THROTTLE
#define STABILIZATION_MFC_MAX_THROTTLE 500
#endif

#if LOG_MFC
#include "modules/loggers/sdlog_chibios.h"
bool log_started = false;
#endif

struct Mfc mfc;
struct MfcParameters mfc_roll;
struct MfcParameters mfc_pitch;
struct MfcParameters mfc_yaw;

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct FloatEulers stab_rc_sp;
struct FloatQuat   q_sp;

int16_t stabilization_mfc_cmd[MFC_NUM_CMD];

int16_t motor_left    = 0; // only for tests
int16_t motor_right   = 0; // only for tests
float counter_test    = 0; // only for tests
float vel_x_gs       = 0; // only for tests
float vel_y_gs       = 0; // only for tests
float vel_z_gs       = 0; // only for tests

/**
 * Function that resets important values upon engaging MFC.
 *
 * Don't reset inputs and filters, because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 * FIXME: Ideally we should detect when coming from something that is not MFC
 */
void stabilization_attitude_mfc_enter(void)
{
  mfc_roll.id     	        = STABILIZATION_MFC_ROLL_ID;
   
  /* Reset incremental time for attitude mfc control loop */
  mfc.start_time = sec_of_sys_time_ticks(sys_time.nb_tick);

  /* Reset heading setpoint to current psi angle */
  stab_att_sp_euler.phi  = stabilization_attitude_get_heading_i();
  mfc_roll.setpoint      = stabilization_attitude_get_heading_f();

  struct FloatEulers *att_eulers = stateGetNedToBodyEulers_f();

  /* MFC roll parameters initialization */
  mfc_roll.setpoint_trajec[2] = att_eulers->phi;
  mfc_roll.setpoint_trajec[1] = att_eulers->phi;
  mfc_roll.setpoint_trajec[0] = att_eulers->phi;  
  mfc_roll.measure  = att_eulers->phi;
  float_vect_zero(mfc_roll.error, 3);
  float_vect_zero(mfc_roll.estimator_num, 3);
  float_vect_zero(mfc_roll.estimator_den, 3);  
  
  /* MFC pitch parameters initialization */
  mfc_pitch.setpoint_trajec[2] = att_eulers->theta;
  mfc_pitch.setpoint_trajec[1] = att_eulers->theta;
  mfc_pitch.setpoint_trajec[0] = att_eulers->theta;
  mfc_pitch.measure  = att_eulers->theta;
  float_vect_zero(mfc_pitch.error, 3);
  float_vect_zero(mfc_pitch.estimator_num, 3);
  float_vect_zero(mfc_pitch.estimator_den, 3);

  /* MFC yaw parameters initialization*/
  mfc_yaw.setpoint_trajec[2] = mfc_yaw.setpoint;
  mfc_yaw.setpoint_trajec[1] = mfc_yaw.setpoint;
  mfc_yaw.setpoint_trajec[0] = mfc_yaw.setpoint;  
  mfc_yaw.measure  = mfc_yaw.setpoint;
  float_vect_zero(mfc_yaw.error, 3);
  float_vect_zero(mfc_yaw.estimator_num, 3);
  float_vect_zero(mfc_yaw.estimator_den, 3);

  /*struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatEulers att_eulers;
  float_eulers_of_quat_zxy(&att_eulers, quat);

  mfc_roll.setpoint_trajec[2] = att_eulers.phi;
  mfc_roll.setpoint_trajec[1] = att_eulers.phi;
  mfc_roll.setpoint_trajec[0] = att_eulers.phi;  
  mfc_roll.measure  = att_eulers.phi;
  float_vect_zero(mfc_roll.error, 3);
  float_vect_zero(mfc_roll.estimator_num, 3);
  float_vect_zero(mfc_roll.estimator_den, 3);  
  
  mfc_pitch.setpoint_trajec[2] = att_eulers.theta;
  mfc_pitch.setpoint_trajec[1] = att_eulers.theta;
  mfc_pitch.setpoint_trajec[0] = att_eulers.theta;
  mfc_pitch.measure  = att_eulers.theta;
  float_vect_zero(mfc_pitch.error, 3);
  float_vect_zero(mfc_pitch.estimator_num, 3);
  float_vect_zero(mfc_pitch.estimator_den, 3);

  mfc_yaw.setpoint_trajec[2] = mfc_yaw.setpoint;
  mfc_yaw.setpoint_trajec[1] = mfc_yaw.setpoint;
  mfc_yaw.setpoint_trajec[0] = mfc_yaw.setpoint;  
  mfc_yaw.measure  = mfc_yaw.setpoint;
  float_vect_zero(mfc_yaw.error, 3);
  float_vect_zero(mfc_yaw.estimator_num, 3);
  float_vect_zero(mfc_yaw.estimator_den, 3);*/
}

//#if PERIODIC_TELEMETRY
static void send_mfc_values(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_MFC(trans, dev, AC_ID,
      &mfc.time,
      &mfc.start_time,
      &mfc_roll.setpoint,
      &mfc_pitch.setpoint,
      &mfc_yaw.setpoint,
      &mfc_roll.setpoint_trajec[0],
      &mfc_pitch.setpoint_trajec[0],
      &mfc_yaw.setpoint_trajec[0],
      &mfc_roll.measure,
      &mfc_pitch.measure,
      &mfc_yaw.measure,
      &mfc_roll.estimator,
      &mfc_pitch.estimator,
      &mfc_yaw.estimator,
      &mfc_roll.error[0],
      &mfc_roll.command[0],
      &stabilization_mfc_cmd[0],
      &actuators_pprz[2],
      &actuators_pprz[3],
      &counter_test,
      &vel_x_gs,
      &vel_y_gs,
      &vel_z_gs); //23
}
//#endif

/**
 * Function that initializes important values upon engaging MFC
 */
void stabilization_attitude_mfc_init(void)
{
  mfc.sample_time 		= 1.f / STABILIZATION_MFC_CONTROL_FREQUENCY;

  // ROLL CONTROL PARAMETERS
  mfc_roll.id     	        = STABILIZATION_MFC_ROLL_ID;
  mfc_roll.time_trajec 	        = STABILIZATION_MFC_ROLL_TIME_TRAJECTORY;
  mfc_roll.use_trajec_sp    	= STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP;
  mfc_roll.int_window 	        = STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW;
  mfc_roll.alpha 		= STABILIZATION_MFC_ROLL_ALPHA;
  mfc_roll.command_filter 	= STABILIZATION_MFC_ROLL_COMMAND_FILTER;
  mfc_roll.kp			= STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN;
  mfc_roll.kd			= STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN;

  // PITCH CONTROL PARAMETERS
  mfc_pitch.id     	        = STABILIZATION_MFC_PITCH_ID;
  mfc_pitch.time_trajec 	= STABILIZATION_MFC_PITCH_TIME_TRAJECTORY;
  mfc_pitch.use_trajec_sp   	= STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP; 
  mfc_pitch.int_window 		= STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW;
  mfc_pitch.alpha 		= STABILIZATION_MFC_PITCH_ALPHA;
  mfc_pitch.command_filter 	= STABILIZATION_MFC_PITCH_COMMAND_FILTER;
  mfc_pitch.kp			= STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN;
  mfc_pitch.kd			= STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN;

  // YAW CONTROL PARAMETERS
  mfc_yaw.id     	        = STABILIZATION_MFC_YAW_ID;
  mfc_yaw.time_trajec 	        = STABILIZATION_MFC_YAW_TIME_TRAJECTORY;
  mfc_yaw.use_trajec_sp    	= STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP;
  mfc_yaw.int_window 	        = STABILIZATION_MFC_YAW_INTEGRATION_WINDOW;
  mfc_yaw.alpha 		= STABILIZATION_MFC_YAW_ALPHA;
  mfc_yaw.command_filter 	= STABILIZATION_MFC_YAW_COMMAND_FILTER;
  mfc_yaw.kp			= STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN;
  mfc_yaw.kd			= STABILIZATION_MFC_YAW_DERIVATIVE_GAIN;
	
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_MFC, send_mfc_values);
#endif
}

/**
 * Function that calculates the failsafe setpoint
 */
void stabilization_attitude_mfc_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  /*int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2); */
}

/**
 * Function that reads roll, pitch, and yaw rc setpoints
*/
void stabilization_attitude_mfc_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  #if USE_EARTH_BOUND_RC_SETPOINT
    stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
  #else
    stabilization_attitude_read_rc_setpoint_eulers_f(&stab_rc_sp, in_flight, in_carefree, coordinated_turn);
  #endif

  /*mfc_roll.setpoint  = stab_rc_sp.psi;
  mfc_pitch.setpoint = 0.5f*M_PI + stab_rc_sp.theta; // Convention fixed-wing
  mfc_yaw.setpoint   = stab_rc_sp.phi;*/

  mfc_roll.setpoint  = stab_rc_sp.phi;
  mfc_pitch.setpoint = stab_rc_sp.theta; // Convention rotorcraft
  mfc_yaw.setpoint   = stab_rc_sp.psi;
}

/**
 * Function that calculate the command with Model-free control commands
*/
void stabilization_attitude_mfc_calc_cmd(bool in_flight, struct MfcParameters *mfc_stt, float measure)
{
  mfc.time = sec_of_sys_time_ticks(sys_time.nb_tick) - mfc.start_time;
  mfc_stt->measure  = measure;

  // 1) Reference filter
  mfc_stt->setpoint_trajec[0] = (mfc_stt->setpoint + (2.f * mfc_stt->time_trajec*mfc_stt->time_trajec + 2.f * mfc_stt->time_trajec)*
  mfc_stt->setpoint_trajec[1] + (-mfc_stt->time_trajec * mfc_stt->time_trajec)*mfc_stt->setpoint_trajec[2]) / 
  (mfc_stt->time_trajec*mfc_stt->time_trajec + 2.f * mfc_stt->time_trajec + 1.f);

  mfc_stt->error[0] = mfc_stt->measure - mfc_stt->setpoint_trajec[0];

  //if(mfc_stt->id==0) // Normalize heading in hover flight mode (phi angle)
  //{
  //  FLOAT_ANGLE_NORMALIZE(mfc_stt->error[0]);   
  //}

  //float dot_setpoint_trajec = (mfc.setpoint_trajec[0] - mfc.setpoint_trajec[1]) / mfc.sample_time;
  float dot_dot_setpoint_trajec = (mfc_stt->setpoint_trajec[0] - 2.f * mfc_stt->setpoint_trajec[1] + mfc_stt->setpoint_trajec[2]) /
  (mfc.sample_time*mfc.sample_time);

  // 2) Control poles (s-p)^2=s^2+as+b  --> Correct the mfc gains with derivative gain in the loop 
  float a = -2.f * mfc_stt->kp;
  float b = -mfc_stt->kp * mfc_stt->kp;

  // ddoty = F + alpha * u + a.dote + b.e

  float sde = -(mfc.time*mfc_stt->error[0] - (mfc.time - mfc.sample_time)*mfc_stt->error[1]) / mfc.sample_time;
  float s2d2e = (mfc.time*mfc.time*mfc_stt->error[0] - 2.f * (mfc.time - mfc.sample_time)*(mfc.time - mfc.sample_time)*mfc_stt->error[1] +
  (mfc.time - 2.f * mfc.sample_time)*(mfc.time - 2.f * mfc.sample_time) * mfc_stt->error[2]) / (mfc.sample_time*mfc.sample_time);
  float sd2e = (mfc.time*mfc.time * mfc_stt->error[0] - (mfc.time - mfc.sample_time)*(mfc.time - mfc.sample_time) * mfc_stt->error[1]) /  
  mfc.sample_time;
  float de = -mfc.time * mfc_stt->error[0];
  float d2e = mfc.time * mfc.time * mfc_stt->error[0];
  float d2u = mfc.time * mfc.time * mfc_stt->command[1];

  // d ^ 2 / ds ^ 2 ->
  float num = 2.f*mfc_stt->error[0] + 4.f*sde + s2d2e - a*(2.f*de+sd2e) - b*(d2e) - mfc_stt->alpha*d2u;
  float den = mfc.time*mfc.time;

  // 3) Filter and estimator F
  mfc_stt->estimator_num[0] = (num + (2.f * mfc_stt->int_window*mfc_stt->int_window + 2.f * mfc_stt->int_window)*mfc_stt->estimator_num[1]+
  (-mfc_stt->int_window*mfc_stt->int_window)*mfc_stt->estimator_num[2]) / (mfc_stt->int_window*mfc_stt->int_window + 
  2.f * mfc_stt->int_window + 1.f);
  mfc_stt->estimator_den[0] = (den + (2.f * mfc_stt->int_window*mfc_stt->int_window + 2.f * mfc_stt->int_window)*mfc_stt->estimator_den[1]+
  (-mfc_stt->int_window*mfc_stt->int_window)*mfc_stt->estimator_den[2]) / (mfc_stt->int_window*mfc_stt->int_window + 
  2.f * mfc_stt->int_window + 1.f);

  float F_k = 0.0f;
  if ((mfc_stt->estimator_den[0] != 0.0f) && (mfc.time > 0.1f))
  {
    F_k = mfc_stt->estimator_num[0] / mfc_stt->estimator_den[0];
  }

  // Command generation
  mfc_stt->command[0] = -F_k / mfc_stt->alpha + dot_dot_setpoint_trajec / mfc_stt->alpha;

  // Saturation
  // U_k = max(Umin, U_k);
  // U_k = min(Umax, U_k);

  // Filter of U eventually necessary
  mfc_stt->command[0] = (mfc_stt->command[0] + (mfc_stt->command_filter - 1.f)*mfc_stt->command[1]) / mfc_stt->command_filter;

  /*mfc_stt->command[0] = (int16_t) TRIM_PPRZ(mfc_stt->command[0]);

  if(in_flight){}

  return mfc_stt->command[0];*/

  if (mfc_stt->command[0] >= 9600.f)
  {
    mfc_stt->command[0] = 9600.f;	
  }
  if (mfc_stt->command[0] <= -9600.f)
  {
    mfc_stt->command[0] = -9600.f;	
  }

  if(in_flight){}

  //float mfc_output[] = {mfc_stt->command[0], F_k};
  mfc_stt->estimator = F_k;


  // Update vectors with previous values
  mfc_stt->setpoint_trajec[2] = mfc_stt->setpoint_trajec[1];
  mfc_stt->setpoint_trajec[1] = mfc_stt->setpoint_trajec[0];
  mfc_stt->estimator_num[2] = mfc_stt->estimator_num[1];
  mfc_stt->estimator_num[1] = mfc_stt->estimator_num[0];
  mfc_stt->estimator_den[2] = mfc_stt->estimator_den[1];
  mfc_stt->estimator_den[1] = mfc_stt->estimator_den[0];
  mfc_stt->error[2] = mfc_stt->error[1];
  mfc_stt->error[1] = mfc_stt->error[0];
  mfc_stt->command[1] = mfc_stt->command[0];

  //return (int16_t) TRIM_PPRZ(9600.f*mfc_stt->command[0]);
}

void stabilization_attitude_mfc_run(bool in_flight)
{
  // Eulers angle measurements
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  //struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  //struct FloatEulers att;
  //float_eulers_of_quat_zxy(&att, quat);

  /*if(att->pitch <= 110*math.pi/180 && qtt->pitch >= 70*math.pi/180){
    in_hover      = true;
    in_transition = false;
    in_forward    = false;
  }*/
  
  // Update MFC commands
  stabilization_attitude_mfc_calc_cmd(in_flight, &mfc_roll, att->phi);
  stabilization_attitude_mfc_calc_cmd(in_flight, &mfc_pitch, att->theta); 
  stabilization_attitude_mfc_calc_cmd(in_flight, &mfc_yaw, att->psi); 

  stabilization_mfc_cmd[0] = (int16_t) TRIM_PPRZ(1000.f*mfc_roll.command[0]);
  stabilization_mfc_cmd[1] = (int16_t) TRIM_PPRZ(9600.f*mfc_pitch.command[0]);
  stabilization_mfc_cmd[2] = (int16_t) TRIM_PPRZ(9600.f*mfc_yaw.command[0]);
  stabilization_mfc_cmd[3] = radio_control.values[RADIO_THROTTLE];

  vel_x_gs = stateGetSpeedNed_f()->x;
  vel_y_gs = stateGetSpeedNed_f()->y;
  vel_z_gs = stateGetSpeedNed_f()->z;
  counter_test = 1.0;
}

/**
 * Function that mixes MFC commands for attitude stabilization 
 * in hovering flight mode (fixedwing setup).
 * 
 * ELEVON LEFT    =    - pitch command ok  -  plus yaw command ok
 * ELEVON RIGHT   =    + pitch command ok  -  plus yaw command ok
 * MOTOR LEFT     =    + nominal thrust ok +  differential thrust (roll cmd) ok
 * MOTOR RIGHT    =    + nominal thrust ok -  differential thrust (roll cmd) ok
*/
void stabilization_attitude_mfc_mixing(void)
{
  //motor_left  = + stabilization_mfc_cmd[0];
  //motor_right = - stabilization_mfc_cmd[0];

  actuators_pprz[0] = - stabilization_mfc_cmd[1] - stabilization_mfc_cmd[2];
  actuators_pprz[1] = + stabilization_mfc_cmd[1] - stabilization_mfc_cmd[2];
  actuators_pprz[2] = + stabilization_mfc_cmd[3] + stabilization_mfc_cmd[0];
  actuators_pprz[3] = + stabilization_mfc_cmd[3] - stabilization_mfc_cmd[0];

  Bound(actuators_pprz[2], 0, 9600);
  Bound(actuators_pprz[3], 0, 9600);

  //actuators_pprz[2] = + stabilization_mfc_cmd[3] + stabilization_mfc_cmd[0]; //Bound(motor_left, 0, 9600);
  //actuators_pprz[3] = + stabilization_mfc_cmd[3] - stabilization_mfc_cmd[0]; //Bound(motor_right, 0, 9600);
}
