/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 *
 * Based on the papers:
 * Cascaded Incremental Nonlinear Dynamic Inversion Control for MAV Disturbance Rejection
 * https://www.researchgate.net/publication/312907985_Cascaded_Incremental_Nonlinear_Dynamic_Inversion_Control_for_MAV_Disturbance_Rejection
 *
 * Gust Disturbance Alleviation with Incremental Nonlinear Dynamic Inversion
 * https://www.researchgate.net/publication/309212603_Gust_Disturbance_Alleviation_with_Incremental_Nonlinear_Dynamic_Inversion
 */
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"

// By MH 
#include "math/wls/wls_alloc.h" 
#include "modules/actuators/actuators.h"


// other variables 

// #define INDI_NUM_ACT 6

// or can be modified to use the structure defined in WLS_alloc.h

// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifdef GUIDANCE_INDI_POS_GAIN
float guidance_indi_pos_gain = GUIDANCE_INDI_POS_GAIN;
#else
float guidance_indi_pos_gain = 0.5;
#endif

#ifdef GUIDANCE_INDI_SPEED_GAIN
float guidance_indi_speed_gain = GUIDANCE_INDI_SPEED_GAIN;
#else
float guidance_indi_speed_gain = 1.8;
#endif

// Call ABI messages acc_sp
#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST 
#endif
abi_event accel_sp_ev;
static void accel_sp_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 *accel_sp);

// RPM
#ifndef GUIDANCE_INDI_ACT_FEEDBACK_ID
#define GUIDANCE_INDI_ACT_FEEDBACK_ID ABI_BROADCAST 
#endif
abi_event act_feedback_ev;
static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);
struct FloatVect3 indi_accel_sp = {0.0f, 0.0f, 0.0f};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
// static void guidance_indi_filter_thrust(void);

#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
#warning GUIDANCE_INDI_THRUST_DYNAMICS is deprecated, use GUIDANCE_INDI_THRUST_DYNAMICS_FREQ instead.
#warning "The thrust dynamics are now specified in continuous time with the corner frequency of the first order model!"
#warning "define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ in rad/s"
#warning "Use -ln(1 - old_number) * PERIODIC_FREQUENCY to compute it from the old value."
#endif
#ifndef GUIDANCE_INDI_THRUST_DYNAMICS_FREQ
#ifndef STABILIZATION_INDI_ACT_FREQ_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ STABILIZATION_INDI_ACT_FREQ_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS_FREQ
#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

// Callback function for RPM
float act_obs_Guidance[INDI_NUM_ACT];
float act_obs_Guidance_rad_sec[INDI_NUM_ACT];

// uint8_t num_act;
// num_act = 6 ; //INDI_NUM_ACT; 



// All parameters that are going to be used in the following functions 
// Strcutures for the filters 
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass yaw_filt;
Butterworth2LowPass actuator_lowpass_filters_Guidance[INDI_NUM_ACT];
struct FloatMat35 {
  float m[3 * 5];
};

// Control loop parameters 
struct FloatMat35 Ga_FA;                        
float du_guidance[5];                           // array to store all control increments from WLS 
float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;
float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;
struct FloatEulers guidance_euler_cmd;
struct ThrustSetpoint thrust_sp;              // Should be a vector of 3 elements normalaized [0;9600]
float thrust_in;
float thrust_dyn = 0.f;
float thrust_act = 0.f;
#define INDI_G_SCALING 1000.0                                 // Taken from stabilization_indi.h ( Should find smarter way to use it) Scaling for the control effectiveness to make it readible

float g_thrust[3][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_THRUST_X,
                                  STABILIZATION_INDI_G1_THRUST_Y,
                                  STABILIZATION_INDI_G1_THRUST};


// This struct should be change because WLS_N_V_MAX are different in the stabilization and guidance loops 
#define NU_MAX 5  // Example constant value (Fx,Fy,Fz,phi,theta)
#define NV_MAX 3  // Example constant value (ax,ay,az)


struct WLS_t wls_guid_p = {
  .nu        = NU_MAX,
  .nv        = NV_MAX,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
  .Wv        =  { 10.f, 10.f, 10.f },
  .Wu        = {1.f,1.f,10.f,1000.f,1000.f},
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

// FIXME make a proper structure for these variables
struct FloatVect3 speed_sp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 sp_accel = {0.0f, 0.0f, 0.0f};

static void guidance_indi_propagate_filters(struct FloatEulers *eulers);

static void guidance_indi_calcG_yxz_FA(struct FloatMat35 *Gm, struct FloatEulers *euler_yxz, float *Thrust_filtered); // By MH
static void guidance_indi_set_wls_settings(struct FloatEulers *euler_yxz, float *Thrust_filtered, float *m, struct FloatEulers *euler_yxz_ref);


// to be changed 

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_indi_guidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &du_guidance[2],
                              &du_guidance[3],
                              &du_guidance[4],
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &speed_sp.x,
                              &speed_sp.y,
                              &speed_sp.z);
}
#endif 



// Main init function.
/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  FLOAT_EULERS_ZERO(guidance_euler_cmd); 
  THRUST_SP_SET_ZERO(thrust_sp);        
  AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);
  AbiBindMsgACT_FEEDBACK(GUIDANCE_INDI_ACT_FEEDBACK_ID, &act_feedback_ev, act_feedback_cb);   // RPM_FEEDBACK From the telemetry 

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_indi_guidance);
#endif
}

/**
 *
 * Call upon entering indi guidance
 * Used to filter phi, theta, thrust
 */
void guidance_indi_enter(void)
{
  /* set nav_heading to current heading */
  nav.heading = stateGetNedToBodyEulers_f()->psi;

  thrust_in = stabilization.cmd[COMMAND_THRUST]; 
  thrust_act = thrust_in; 

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
  thrust_dyn = GUIDANCE_INDI_THRUST_DYNAMICS;
#else
  thrust_dyn = 1-exp(-GUIDANCE_INDI_THRUST_DYNAMICS_FREQ/PERIODIC_FREQUENCY);
#endif 
#endif 
  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in); // mainly this won't be used 
  // intializing filters for the rotor speed 
  for (int i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters_Guidance[i], tau, sample_time, 0.0);
  }
}

/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accel_sp, float heading_sp)
{
  // Compute Eulers from quaternion
  struct FloatEulers euler_yxz;
  struct FloatEulers euler_yxz_ref;   // Fix me from RC or trajectory generation loop for now it will be fixed to zeros
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&euler_yxz, statequat);    // Should be changed to have the same orientation as the matrix used
  
  // Compute the filtered thrust from measured actuators 
  float Thrust_filtered[3] = {0.0f}; 
  float actuator_state_filt_vect_prev[INDI_NUM_ACT];
  for (int i = 0; i < INDI_NUM_ACT; i++) {
    update_butterworth_2_low_pass(&actuator_lowpass_filters_Guidance[i], act_obs_Guidance_rad_sec[i]);
    actuator_state_filt_vect_prev[i] = actuator_lowpass_filters_Guidance[i].o[1];
  }
  for (int i =0;i < 3; i++) {
      for (int j =0;j < INDI_NUM_ACT; j++) {
        Thrust_filtered[i] += (g_thrust[i][j]*actuator_state_filt_vect_prev[j])/INDI_G_SCALING; // should I squarte the actuator_lowpass_filters_Guidance
      }
  }
  
  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&euler_yxz);

  //Calculate matrix of partial derivatives
  guidance_indi_calcG_yxz_FA(&Ga_FA, &euler_yxz, Thrust_filtered); // By MH

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y - filt_accel_ned[1].o[0], sp_accel.z - filt_accel_ned[2].o[0]};

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  //Calculate roll,pitch, Tx, Ty, and Tz increments 
  float m = 1.0f;  // Define the weighting parameters
  wls_guid_p.v[0] = m*a_diff.x;   
  wls_guid_p.v[1] = m*a_diff.y;
  wls_guid_p.v[2] = m*a_diff.z;
  // To be taken from an RC later 
  euler_yxz_ref.phi = 0;
  euler_yxz_ref.theta= 0;
  euler_yxz_ref.psi = 0;
  // function called to change setting of the WLS in the structure wls_guid_p
  guidance_indi_set_wls_settings(&euler_yxz,Thrust_filtered, &m, &euler_yxz_ref);

  /*
  wls_alloc takes as an input:
  1- Structure guidance containing the main parameters of the WLS.
  2- The control effectivness matric Ga_FA
  3- u_guess 
  4- W_init Initial working set, if known
  5- imax Max number of iterations
  */ 
 
    wls_alloc(&wls_guid_p,(float**)(&Ga_FA.m), 0, 0, 10);
    float INDI_NUM_INPUT;
    INDI_NUM_INPUT = 5;       // should be defined in the configuration file 
    for (int i = 0; i < INDI_NUM_INPUT; i++) {
    du_guidance [i] = wls_guid_p.u[i];
  }
  //
  guidance_euler_cmd.theta = pitch_filt.o[0] + du_guidance [0];
  guidance_euler_cmd.phi = roll_filt.o[0] +  du_guidance [1];
  guidance_euler_cmd.psi = heading_sp;

  // GUIDANCE_INDI_SPECIFIC_FORCE_GAIN will not be defined so getting rid
  // of this for now.
  float thrust_vect[3];
  thrust_vect[0] = du_guidance[2];   // dTx
  thrust_vect[1] = du_guidance[3];   // dTy
  thrust_vect[2] = du_guidance[4];   // dTz

  // specific force not defined, return required increment
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);    //[dTx,dTy,dTz] since format is THRUST_SP_FLOAT it is in PPRz 

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);

  //set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);

  return stab_sp_from_quat_f(&q_sp);
}

struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 accel_sp = { 0 };

  struct FloatVect3 speed_fb;


  if (h_mode == GUIDANCE_INDI_H_ACCEL) {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.x = 0.;
    speed_fb.y = 0.;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (h_mode == GUIDANCE_INDI_H_SPEED) {
      speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    else { // H_POS
      pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
      pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
      speed_sp.x = pos_err.x * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      speed_sp.y = pos_err.y * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    speed_fb.x = (speed_sp.x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
    speed_fb.y = (speed_sp.y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
  }

  if (v_mode == GUIDANCE_INDI_V_ACCEL)  {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.z = 0;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (v_mode == GUIDANCE_INDI_V_SPEED) {
      speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    else { // V_POS
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      speed_sp.z = pos_err.z * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    speed_fb.z = (speed_sp.z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
  }

  accel_sp.x = speed_fb.x + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  accel_sp.y = speed_fb.y + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  accel_sp.z = speed_fb.z + ACCEL_FLOAT_OF_BFP(gv->zdd_ref);

  return guidance_indi_run(&accel_sp, gh->sp.heading);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 */
void guidance_indi_propagate_filters(struct FloatEulers *eulers)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers->phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers->theta);
}

/**
 * @param Ga_FA array to write the matrix to [3x5]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dtheta,dphi,dTx,dTy,dTz]
 */
void guidance_indi_calcG_yxz_FA(struct FloatMat35 *Gm, struct FloatEulers *euler_yxz, float *Thrust_filtered) // By MH
{
  // Euler Angles
  float sphi = sinf(euler_yxz->phi);
  float cphi = cosf(euler_yxz->phi);
  float stheta = sinf(euler_yxz->theta);
  float ctheta = cosf(euler_yxz->theta);
  float cpsi  = cosf(euler_yxz->psi);
  float spsi  = sinf(euler_yxz->psi);

  // Estimated Thrust 
  float Tx =  Thrust_filtered[0];
  float Ty =  Thrust_filtered[1];
  float Tz =  Thrust_filtered[2];

  // dtheta
  RMAT_ELMT(*Gm, 0, 0) =  Tz * cphi * cpsi * ctheta - Tx * cpsi * stheta + Ty * cpsi * ctheta * sphi;
  RMAT_ELMT(*Gm, 1, 0) =  Tz * cphi * ctheta * spsi - Tx * spsi * stheta + Ty * ctheta * sphi * spsi;
  RMAT_ELMT(*Gm, 2, 0) = -Tx * ctheta - Tz * cphi * stheta - Ty * sphi * stheta;
  // dphi 
  RMAT_ELMT(*Gm, 0, 1) =  Ty * (sphi * spsi + cphi * cpsi * stheta) + Tz * (cphi * spsi - cpsi * sphi * stheta);
  RMAT_ELMT(*Gm, 1, 1) = -Ty * (cpsi * sphi - cphi * spsi * stheta) - Tz * (cphi * cpsi + sphi * spsi * stheta);
  RMAT_ELMT(*Gm, 2, 1) =  Ty*ctheta*cphi -ctheta * sphi * Tz;
  // dTx 
  RMAT_ELMT(*Gm, 0, 2) =  cpsi * ctheta;
  RMAT_ELMT(*Gm, 1, 2) =  ctheta * spsi;
  RMAT_ELMT(*Gm, 2, 2) = -stheta;
  // dTy 
  RMAT_ELMT(*Gm, 0, 3) =  cpsi*sphi*stheta - cphi*spsi;
  RMAT_ELMT(*Gm, 1, 3) =  cphi*cpsi + sphi*spsi*stheta;
  RMAT_ELMT(*Gm, 2, 3) =  ctheta*sphi;
  // dTz 
  RMAT_ELMT(*Gm, 0, 4) =  sphi * spsi + cphi * cpsi * stheta;
  RMAT_ELMT(*Gm, 1, 4) =  cphi * spsi * stheta - cpsi * sphi;
  RMAT_ELMT(*Gm, 2, 4) =  cphi * ctheta;
}

void guidance_indi_set_wls_settings(struct FloatEulers *euler_yxz, float *Thrust_filtered, float *m, struct FloatEulers *euler_yxz_ref) 
{
  float grav = 9.81f;  // gravitational constant
  // Set lower limits
  wls_guid_p.u_min[0] = -guidance_indi_max_bank - euler_yxz->theta; //theta
  wls_guid_p.u_min[1] = -guidance_indi_max_bank - euler_yxz->phi;   //phi
  wls_guid_p.u_min[2] = -1  -  Thrust_filtered[0]; //Tx
  wls_guid_p.u_min[3] = -1  -  Thrust_filtered[1]; //Ty
  wls_guid_p.u_min[4] = -40 -  Thrust_filtered[3]; //Tz
  // Set upper limits limits
  wls_guid_p.u_max[0] =   guidance_indi_max_bank - euler_yxz->theta; //theta
  wls_guid_p.u_max[1] =   guidance_indi_max_bank - euler_yxz->phi; //phi
  wls_guid_p.u_max[2] =   1  -  Thrust_filtered[0]; //Tx
  wls_guid_p.u_max[3] =   1 -  Thrust_filtered[0]; //Tx
  wls_guid_p.u_min[4] =   (*m)*grav  -  Thrust_filtered[0]; //Tx

  // Set prefered states
  wls_guid_p.u_pref[0] =  euler_yxz_ref->theta - euler_yxz->theta;      // prefered delta theta
  wls_guid_p.u_pref[1] =  euler_yxz_ref->phi - euler_yxz->phi;      // prefered delta phi
  wls_guid_p.u_pref[2] =  0;      // prefred Tx
  wls_guid_p.u_pref[3] =  0;      // prefered Ty
  wls_guid_p.u_pref[4] =  0;      // prefered Tz
}


/**
 * ABI callback that obtains the acceleration setpoint from telemetry
 * flag: 0 -> 2D, 1 -> 3D
 */
static void accel_sp_cb(uint8_t sender_id __attribute__((unused)), uint8_t flag, struct FloatVect3 *accel_sp)
{
  if (flag == 0) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp_set_2d = true;
    time_of_accel_sp_2d = get_sys_time_float();
  } else if (flag == 1) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp.z = accel_sp->z;
    indi_accel_sp_set_3d = true;
    time_of_accel_sp_3d = get_sys_time_float();
  }
}

/**
 * ABI callback that obtains the RPM Motors 
 */
static void act_feedback_cb(uint8_t sender_id UNUSED, struct act_feedback_t *feedback, uint8_t num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++) {
    // Sanity check that index is valid
    if (feedback[i].idx < INDI_NUM_ACT && feedback[i].set.rpm) {
      int8_t idx = feedback[i].idx;
      act_obs_Guidance[idx] = (feedback[i].rpm - get_servo_min_PWM(idx));
      act_obs_Guidance_rad_sec[idx] = feedback[i].rpm * 0.10472; // rpm ro rad/sec
      act_obs_Guidance[idx] *= (MAX_PPRZ / (float)(get_servo_max_PWM(idx) - get_servo_min_PWM(idx)));
      Bound(act_obs_Guidance[idx], 0, MAX_PPRZ);
    }
  }
}


#if GUIDANCE_INDI_USE_AS_DEFAULT
// guidance indi control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_indi_enter();
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

static struct VerticalGuidance *_gv = &guidance_v;
static enum GuidanceIndi_VMode _v_mode = GUIDANCE_INDI_V_POS;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_POS, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_SPEED, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_ACCEL, _v_mode);
}

struct ThrustSetpoint guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_POS;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_SPEED;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_ACCEL;
  return thrust_sp;
}

#endif

