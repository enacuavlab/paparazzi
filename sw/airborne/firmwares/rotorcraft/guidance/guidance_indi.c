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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
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

// BY MH 
#include "math/wls/wls_alloc.h" 
#include "modules/actuators/actuators.h"

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
// ABI messages Sender 
#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST
#endif
abi_event accel_sp_ev;
static void accel_sp_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 *accel_sp);

// ------------------------ABI message RPM MH -----------------------------//
#ifndef GUIDANCE_INDI_ACT_FEEDBACK_ID
#define GUIDANCE_INDI_ACT_FEEDBACK_ID ABI_BROADCAST 
#endif
// abi_event act_feedback_ev;
// static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);
// ------------------------------------------------------------------------//

struct FloatVect3 indi_accel_sp = {0.0f, 0.0f, 0.0f};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

// FIXME make a proper structure for these variables
struct FloatVect3 speed_sp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 sp_accel = {0.0f, 0.0f, 0.0f};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

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

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float thrust_dyn = 0.f;
float thrust_act = 0.f;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass psi_filt;
Butterworth2LowPass thrust_filt;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 control_increment; // [dtheta, dphi, dthrust]

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;

struct FloatEulers guidance_euler_cmd;
struct ThrustSetpoint thrust_sp;
float thrust_in;
float thrust_vect[3];

//----------------------- BY MH --------------------------  // 
float act_obs_Guidance[INDI_NUM_ACT];
float act_obs_Guidance_rad_sec[INDI_NUM_ACT];
struct FloatMat35 {
  float m[3 * 5];
};

#define NU_MAX 6    // Example constant value // [dtheta, dphi, dthrust, dtx , dty]
#define NV_MAX 3    // Example constant value (ax,ay,az)
float Euler_ref_tester[3];
float du_guidance[NU_MAX];              
float Ga_FA[NV_MAX][NU_MAX];
float *Bwls_gih[NV_MAX];

#define Guidance_INDI_ALLOCATION_PSEUDO_INVERSE FALSE
#if Guidance_INDI_ALLOCATION_PSEUDO_INVERSE
static void calc_GFA_pseudo_inv(void);
float Ga_FA_pseudo_inv[NU_MAX][NV_MAX];
float Ga_FA_trans_mult[NV_MAX][NV_MAX];
float Ga_FAinv[INDI_OUTPUTS][INDI_OUTPUTS];
#endif

struct WLS_t wls_guid_p = {
  .nu        = NU_MAX,
  .nv        = NV_MAX,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
  .Wv        = {100.f, 100.f, 100.f},         // x,y,z
  .Wu        = {10.f,10.f,0.f,0.f,0.f,10.f}, // minimize the control input (thetq,phi,Tx,Ty,Tz,psi)
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};
// static void guidance_indi_calcG_xyz_FA(float Gmat[NV_MAX][NU_MAX], struct FloatEulers *euler_yxz, float *Thrust_filtered_Guidance); // By MH
static void guidance_indi_calcG_yxz_FA(float Ga_FA[NV_MAX][NU_MAX], struct FloatEulers *euler_yxz, float *Thrust_filtered_Guidance); // By MH
static void guidance_indi_set_wls_settings(struct FloatEulers *euler_yxz, float *Thrust_filtered_Guidance, float m, struct FloatEulers *euler_yxz_ref, float heading_sp);
//------------------------------------------------------------// 
static void guidance_indi_propagate_filters(struct FloatEulers *eulers);
//static void guidance_indi_calcG(struct FloatMat33 *Gmat);
//static void guidance_indi_calcG_yxz(struct FloatMat33 *Gmat, struct FloatEulers *euler_yxz, float *Thrust_filtered_Guidance);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_indi_guidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &Euler_ref_tester[0],
                              &Euler_ref_tester[1], 
                              &du_guidance[2], 
                              &actuator_state_filt_vect[0], 
                              &actuator_state_filt_vect[1], 
                              &actuator_state_filt_vect[2],
                              &actuator_state_filt_vect[3],
                              &actuator_state_filt_vect[4],
                              &actuator_state_filt_vect[5],
                              &Thrust_filtered[0],
                              &Thrust_filtered[1],
                              &Thrust_filtered[2]);
}
#endif
/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  FLOAT_EULERS_ZERO(guidance_euler_cmd);
  THRUST_SP_SET_ZERO(thrust_sp);
  AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);
  //------------- BY MH Call back ACT_FEEDBACK------------------//
  // AbiBindMsgACT_FEEDBACK(GUIDANCE_INDI_ACT_FEEDBACK_ID, &act_feedback_ev, act_feedback_cb);   // RPM_FEEDBACK From the telemetry 
  //------------------------------------------------------------//
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_indi_guidance);
#endif
}

/**
 *
 * Call upon entering indi guidance
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
#endif //GUIDANCE_INDI_THRUST_DYNAMICS
#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&psi_filt, tau, sample_time, stateGetNedToBodyEulers_f()->psi);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
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
  float m = 0.65;  // Define the weighting parameters
  struct FloatEulers euler_yxz;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&euler_yxz, statequat);

  float Thrust_filtered_Guidance[3];
  Thrust_filtered_Guidance[0] = Thrust_filtered[0];
  Thrust_filtered_Guidance[1] = Thrust_filtered[1];
  Thrust_filtered_Guidance[2] = Thrust_filtered[2];
  struct FloatEulers euler_yxz_ref;        // ! FIX ME From RC          

  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&euler_yxz);

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y - filt_accel_ned[1].o[0], sp_accel.z - filt_accel_ned[2].o[0]};
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);
  wls_guid_p.v[0] = m*a_diff.x;   
  wls_guid_p.v[1] = m*a_diff.y;
  wls_guid_p.v[2] = m*a_diff.z;
  // ! FIX ME : To be taken from an RC later or a planer
  euler_yxz_ref.phi = 0;
  euler_yxz_ref.theta = 0.1*0;
  euler_yxz_ref.psi = 0;//heading_sp;


  #if GUIDANCE_INDI_RC_SWITCH_EULER
    // euler_yxz_ref.phi =   (radio_control.values[RADIO_PITCH] / 9600.0);
    // euler_yxz_ref.theta = (radio_control.values[RADIO_ROLL] / 9600.0);
    Euler_ref_tester[0] = (radio_control.values[RADIO_ROLL] / 9600.0)*0.5;
    Euler_ref_tester[1] = (radio_control.values[RADIO_PITCH] / 9600.0)*0.5;
    euler_yxz_ref.phi  = Euler_ref_tester[0];
    euler_yxz_ref.theta = Euler_ref_tester[1];
  #endif


  guidance_indi_calcG_yxz_FA(Ga_FA, &euler_yxz, Thrust_filtered_Guidance); 

  #if Guidance_INDI_ALLOCATION_PSEUDO_INVERSE
  calc_GFA_pseudo_inv();
  for (int i = 0; i < NU_MAX; i++) {
      for (int j = 0; j < NV_MAX; j++) {
        du_guidance [i] = + Ga_FA_pseudo_inv[i][j]*wls_guid_p.v[j];
      }
  }  
  #else
  guidance_indi_set_wls_settings(&euler_yxz,Thrust_filtered_Guidance, m, &euler_yxz_ref , heading_sp);
  for (int i = 0; i < NV_MAX; i++) {
      Bwls_gih[i] = Ga_FA[i];
  }
  wls_alloc(&wls_guid_p,Bwls_gih, 0, 0, 10);
  for (int i = 0; i < NU_MAX; i++) {
    du_guidance [i] = wls_guid_p.u[i];
  }
  #endif
  guidance_euler_cmd.theta = (pitch_filt.o[0] + du_guidance[0]);  
  guidance_euler_cmd.phi = (roll_filt.o[0] + du_guidance[1]);     
  guidance_euler_cmd.psi = (psi_filt.o[0] + du_guidance[5]);
  thrust_vect[0] = du_guidance[3];   // (TX)
  thrust_vect[1] = du_guidance[4];   // (TY)
  thrust_vect[2] = du_guidance[2];   // (TZ)
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);

  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);
  //set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);
  return stab_sp_from_quat_f(&q_sp);
}

struct FloatVect3 WEAK guidance_indi_controller(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
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

  return accel_sp;
}

struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 accel_sp = guidance_indi_controller(in_flight, gh, gv, h_mode, v_mode);
  return guidance_indi_run(&accel_sp, gh->sp.heading);
}


#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + thrust_dyn * (thrust_in - thrust_act);

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}
#endif

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
  update_butterworth_2_low_pass(&psi_filt, eulers->psi);

}

#if Guidance_INDI_ALLOCATION_PSEUDO_INVERSE
/**
 * Function that calculates the pseudo-inverse of (G1+G2).
 * Make sure to sum of G1 and G2 before running this!
 */
void calc_GFA_pseudo_inv(void)
{
  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose INDI_OUTPUTSxnum_act x num_actxINDI_OUTPUTS
  float element = 0;
  int8_t row;
  int8_t col;
  int8_t i;
  for (row = 0; row < NV_MAX; row++) {
    for (col = 0; col < NV_MAX; col++) {
      element = 0;
      for (i = 0; i < NU_MAX; i++) {
        element = element + Ga_FA[row][i] * Ga_FA[col][i];
      }
      Ga_FA_trans_mult[row][col] = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  float_vect_scale(Ga_FA_trans_mult[0], 1000.0, NV_MAX * NV_MAX);

  //inverse of 4x4 matrix
  float_mat_inv_4d(Ga_FAinv, Ga_FA_trans_mult);

  //scale back
  float_vect_scale(Ga_FAinv[0], 1000.0, NV_MAX * NV_MAX);

  //G1G2'*G1G2inv
  //calculate matrix multiplication INDI_NUM_ACTxINDI_OUTPUTS x INDI_OUTPUTSxINDI_OUTPUTS
  for (row = 0; row < NU_MAX; row++) {
    for (col = 0; col < NV_MAX; col++) {
      element = 0;
      for (i = 0; i < INDI_OUTPUTS; i++) {
        element = element + Ga_FA[i][row] * Ga_FAinv[col][i];
      }
      Ga_FA_pseudo_inv[row][col] = element;
    }
  }
}
#endif

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
/*static void act_feedback_cb(uint8_t sender_id UNUSED, struct act_feedback_t *feedback, uint8_t num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++) {
    // Sanity check that index is valid
    if (feedback[i].idx < INDI_NUM_ACT && feedback[i].set.rpm) {
      int8_t idx = feedback[i].idx;
      act_obs_Guidance[idx] = (feedback[i].rpm - get_servo_min_DSHOT(idx)); //get_servo_idx_DSHOT
      act_obs_Guidance_rad_sec[idx] = feedback[i].rpm * 0.10472; // rpm ro rad/sec
      act_obs_Guidance[idx] *= (MAX_PPRZ / (float)(get_servo_max_DSHOT(idx) - get_servo_min_DSHOT(idx)));
      Bound(act_obs_Guidance[idx], 0, MAX_PPRZ);
    }
  }
}*/

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

