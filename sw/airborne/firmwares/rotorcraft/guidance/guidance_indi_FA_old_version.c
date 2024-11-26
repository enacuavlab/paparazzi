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

// This struct should be change because WLS_N_V_MAX are different in the stabilization and guidance loops 
#define NU_MAX 5  // Example constant value
#define NV_MAX 3  // Example constant value
// or can be modified to use the structure deined in WLS_lloc.h
struct guidance_wls_structure {
    float nu;                    // number of incremental inputs {theta phi Tx Ty Tz}
    float nv;                    // number of controlled axes {a_xrr a_yrr a_zrr}
    float gamma_sq;              // weighting factor WLS
    float v[NU_MAX];             // Pseudo Control Vector
    float u[NV_MAX];             // Allocation of Controls
    float Wv[NV_MAX];            // Wv weights on the controlled axes
    float Wu[NU_MAX];            // Aim is to mainly focus on minimizing the Euler's error
    float u_pref[NU_MAX];        // TO be adjusted
    float u_min[NU_MAX];         // TO be adjusted
    float u_max[NU_MAX];         // TO be adjusted
    float PC;
    float SC;
    int iter;                    // TO be adjusted
};
//
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

#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST
#endif
abi_event accel_sp_ev;
static void accel_sp_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 *accel_sp);

// RPM Feedback from the dshot of each motor  Should check if using ABI_BROADCAST Works 
#ifndef GUIDANCE_INDI_ACT_FEEDBACK_ID
#define GUIDANCE_INDI_ACT_FEEDBACK_ID ABI_BROADCAST 
#endif

uint8_t num_act;
num_act = 6; // Having six actuators 
abi_event act_feedback_ev;
static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);

struct FloatVect3 indi_accel_sp = {0.0f, 0.0f, 0.0f};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

// FIXME make a proper structure for these variables
struct FloatVect3 speed_sp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 sp_accel = {0.0f, 0.0f, 0.0f};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void     (void);

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

// By ME 
struct FloatMat35{
  float m[3 * 5];
};
struct FloatVect5{
    float x;
    float y;
    float z;
    float w;
    float v;
};
//

float thrust_dyn = 0.f;
float thrust_act = 0.f;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass yaw_filt;
// struct FloatMat33 Ga;
// struct FloatMat33 Ga_inv;
// struct FloatVect3 control_increment;            // [dtheta, dphi, dthrust]

// By ME
// Need to add here filtered thrust (or filtered Omega_motor)
struct FloatMat35 Ga_FA;                        
struct FloatVect5 control_increment_FA;         // [dtheta, dphi, dTx,dTy,dTz] either use this or use du_guidance
float du_guidance[5];                           // array to store all control increments from WLS 
//

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;

struct FloatEulers guidance_euler_cmd;
struct ThrustSetpoint thrust_sp;              // Should be a vector of 3 elements normalaized [0;9600]
float thrust_in;
struct FloatVect3 Thrust_filtered;            // [Tx,Ty,Tz] esrimated by G*u 

static void guidance_indi_propagate_filters(struct FloatEulers *eulers);

static void guidance_indi_calcG_yxz_FA(struct FloatMat35 *Gmat, struct FloatEulers *euler_yxz, struct FloatVect3 *Thrust_filtered);

// to be changed 
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_indi_guidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &control_increment.x,
                              &control_increment.y,
                              &control_increment.z,
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &speed_sp.x,
                              &speed_sp.y,
                              &speed_sp.z);
}
#endif

// other variables 
#define INDI_NUM_ACT 6
float act_obs[INDI_NUM_ACT];

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

  thrust_in = stabilization.cmd[COMMAND_THRUST]; // Is it the thrust in motor frame ??

  thrust_act = thrust_in; // Thrust in the motor frame 

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
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
}

/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
// All the guidance code is defined in this function 
struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accel_sp, float heading_sp)
{
  // Compute Eulers from quaternion
  struct FloatEulers eulers_yxz;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&eulers_yxz, statequat);
  
  // Compute the filtered thrust from measured actuators 
  struct FloatVect3 *Thrust_filtered //

  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&eulers_yxz);

  //Calculate matrix of partial derivatives
  guidance_indi_calcG_yxz_FA( &Ga_FA, &euler_yxz, &Thrust_filtered) // By MH

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y - filt_accel_ned[1].o[0], sp_accel.z - filt_accel_ned[2].o[0]};

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  //Calculate roll,pitch, Tx, Ty, and Tz increments 
  float m = 1;                        // mass of the drone in KG
  struct guidance_wls_structure guidance;
  // Define the weighting parameters
  guidance.gamma_sq = 1000;
  guidance.nu = 5;
  guidance.nv = 3;
  //
  guidance.Wv[0] = 10;
  guidance.Wv[1] = 10;
  guidance.Wv[2] = 10;
  //
  guidance.Wu[0] = 1;
  guidance.Wu[1] = 1;
  guidance.Wu[2] = 10;
  guidance.Wu[3] = 1000;
  guidance.Wu[4] = 1000;   // To be adjusted as needed 
  //
  guidance.u_pref[0] = 0;
  guidance.u_pref[1] = 0;
  guidance.u_pref[2] = 0;
  guidance.u_pref[3] = 0; // To be adjusted as needed 
  guidance.u_pref[4] = 0;// To be adjusted as needed 

  //
  guidance.umin[0] =  -1.5;      // giving any value for theta and phi min and max 
  guidance.umin[1] =  -1.5;
  guidance.umin[2] =  -1 - Thrust_filtered[0];
  guidance.umin[3] =  -1 - Thrust_filtered[1];
  guidance.umin[4] =  -50 - Thrust_filtered[2];
  //
  guidance.umax[0] =  1.5;      // giving any value for theta and phi min and max 
  guidance.umax[1] =  1.5;
  guidance.umax[2] =  1 - Thrust_filtered[0];
  guidance.umax[3] =  1 - Thrust_filtered[1];
  guidance.umax[4] =  10 - Thrust_filtered[2];
  //
  guidance.PC = 0.0;
  guidance.SC = 0.0;
  guidance.iter = 50;
  // input to the guidance is m*a_err
  guidance.v[0] = m*a_diff.x;   
  guidance.v[1] = m*a_diff.y;
  guidance.v[2] = m*a_diff.z;
  

  /*
  wls_alloc takes as an input:
  1- Structure guidance containing the main parameters of the WLS.
  2- The control effectivness matric Ga_FA
  3- u_guess 
  4- W_init Initial working set, if known
  5- imax Max number of iterations
  */ 
  wls_alloc(&guidance, &Ga_FA, 0, 0, 10);
    INDI_NUM_INPUT = 5;       //[theta;phi;Tx;Ty;Tz]
    for (i = 0; i < INDI_NUM_INPUT; i++) {
    du_guidance [i] = guidance.u[i];
  }
  //
  guidance_euler_cmd.theta = pitch_filt.o[0] + du_guidance [0];
  guidance_euler_cmd.phi = roll_filt.o[0] +  du_guidance [1];
  guidance_euler_cmd.psi = heading_sp;

  // GUIDANCE_INDI_SPECIFIC_FORCE_GAIN will not be defined so getting rid
  // of this for now.
  float thrust_vect[3];
  thrust_vect[0] = du_guidance[2];   // Tx
  thrust_vect[1] = du_guidance[3];   // Ty
  thrust_vect[2] = du_guidance[4];   // Tz

  // specific force not defined, return required increment
  // should be containing at the end 3d vector increments [Tx,Ty,Tz]
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);   
  //

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);

  //set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);

  return stab_sp_from_quat_f(&q_sp);
}

// Function that computes the acceleration_sp 
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

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + thrust_dyn * (thrust_in - thrust_act); // How it is thrust_act , shouldn't be multiplied by G?

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act); // Thrust_filtered = thrust_filt*G!!!!!!
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
}


/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dtheta,dphi,dTx,dTy,dTz]
 */
void guidance_indi_calcG_yxz_FA(struct FloatMat35 *Gmat, struct FloatEulers *euler_yxz, struct FloatVect3 *Thrust_filtered) // By MH
{
  // Euler Angles
  float sphi = sinf(euler_yxz->phi);
  float cphi = cosf(euler_yxz->phi);
  float stheta = sinf(euler_yxz->theta);
  float ctheta = cosf(euler_yxz->theta);
  float cpsi  = cosf(euler_yxz->psi);
  float spsi  = sinf(euler_yxz->psi);

  // Estimated Thrust 
  float Tx =  Thrust_filtered->x;
  float Ty =  Thrust_filtered->y;
  float Tz =  Thrust_filtered->z;

  // dtheta
  RMAT_ELMT(*Gmat, 0, 0) =  Tz * cphi * cpsi * ctheta - Tx * cpsi * stheta + Ty * cpsi * ctheta * sphi;
  RMAT_ELMT(*Gmat, 1, 0) =  Tz * cphi * ctheta * spsi - Tx * spsi * stheta + Ty * ctheta * sphi * spsi;
  RMAT_ELMT(*Gmat, 2, 0) = -Tx * ctheta - Tz * cphi * stheta - Ty * sphi * stheta;
  // dphi 
  RMAT_ELMT(*Gmat, 0, 1) =  Ty * (sphi * spsi + cphi * cpsi * stheta) + Tz * (cphi * spsi - cpsi * sphi * stheta);
  RMAT_ELMT(*Gmat, 1, 1) = -Ty * (cpsi * sphi - cphi * spsi * stheta) - Tz * (cphi * cpsi + sphi * spsi * stheta);
  RMAT_ELMT(*Gmat, 2, 1) = -ctheta * sphi * T;
  // dTx 
  RMAT_ELMT(*Gmat, 0, 2) =  cpsi * ctheta;
  RMAT_ELMT(*Gmat, 1, 2) =  ctheta * spsi;
  RMAT_ELMT(*Gmat, 2, 2) = -stheta;
  // dTy 
  RMAT_ELMT(*Gmat, 0, 3) =  cpsi*sphi*stheta - cphi*spsi;
  RMAT_ELMT(*Gmat, 1, 3) =  cphi*cpsi + sphi*spsi*stheta;
  RMAT_ELMT(*Gmat, 2, 3) =  ctheta*sphi;
  // dTz 
  RMAT_ELMT(*Gmat, 0, 4) =  sphi * spsi + cphi * cpsi * stheta;
  RMAT_ELMT(*Gmat, 1, 4) =  cphi * spsi * stheta - cpsi * sphi;
  RMAT_ELMT(*Gmat, 2, 4) =  cphi * ctheta;
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
      act_obs[idx] = (feedback[i].rpm - get_servo_min(idx));
      act_obs[idx] *= (MAX_PPRZ / (float)(get_servo_max(idx) - get_servo_min(idx)));
      Bound(act_obs[idx], 0, MAX_PPRZ);
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

