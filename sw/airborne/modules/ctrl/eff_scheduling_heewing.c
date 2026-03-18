/*
 * Copyright (C) 2025 Ramon Revilla Bouso <ramonrevilla21@gmail.com>
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

/** @file "modules/ctrl/eff_scheduling_heewing.c"
 * The control effectiveness scheduler for the Heewing T1 Ranger VTOL
 */

#include "modules/ctrl/eff_scheduling_heewing.h"
#include "math/pprz_isa.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

// Tilt position in forward flight [pprz]
#ifndef ESH_TILT_FORWARD
#define ESH_TILT_FORWARD 0
#endif

// Tilt vertical position for hovering [pprz]
#ifndef ESH_TILT_VERTICAL
#define ESH_TILT_VERTICAL 8700
#endif

// Motor idle position [pprz]
#ifndef ESH_MOTOR_IDLE
#define ESH_MOTOR_IDLE 800
#endif

// Max tilt diff [pprz]
#ifndef ESH_TILT_DIFF_MAX
#define ESH_TILT_DIFF_MAX 900
#endif

// Max tilt angle [rad]
#ifndef ESH_TILT_ANGLE_MAX
#define ESH_TILT_ANGLE_MAX RadOfDeg(99.3f)
#endif

// Max motor thrust [Newtons]
#ifndef ESH_MOTOR_THRUST_MAX
#define ESH_MOTOR_THRUST_MAX 3.5f
#endif

// Max airspeed (bound airspeed data) [m/s]
#ifndef ESH_AIRSPEED_MAX
#define ESH_AIRSPEED_MAX 30.f
#endif

// Default mechanical characteristics [m]
// based on T1 Ranger

#ifndef ESH_CHORD
#define ESH_CHORD 0.15f
#endif

#ifndef ESH_SPAN
#define ESH_SPAN 0.73f
#endif

#ifndef ESH_MR_DX
#define ESH_MR_DX 0.09f
#endif

#ifndef ESH_MR_DY
#define ESH_MR_DY 0.12f
#endif

#ifndef ESH_ML_DX
#define ESH_ML_DX 0.09f
#endif

#ifndef ESH_ML_DY
#define ESH_ML_DY 0.12f
#endif

#ifndef ESH_MB_DX
#define ESH_MB_DX 0.25f
#endif

#ifndef ESH_MB_DY
#define ESH_MB_DY 0.0f
#endif

// Matrix size and axis names

#define ESH_EFF_MAT_COLS_NB 7
#define ESH_EFF_MAT_ROWS_NB 5

#define ESH_P 0 // X body axis (angular acceleration)
#define ESH_Q 1 // Y body axis (angular acceleration)
#define ESH_R 2 // Z body axis (angular acceleration)
#define ESH_W 3 // Z body axis (linear acceleration)
#define ESH_U 4 // X body axis (linear acceleration)

#define ESH_CMD_MOTOR_R   0 // Motor Right
#define ESH_CMD_MOTOR_L   1 // Motor Left
#define ESH_CMD_MOTOR_B   2 // Motor Back
#define ESH_CMD_TILT_R    3 // Tilt motor right
#define ESH_CMD_TILT_L    4 // Tilt motor left
#define ESH_CMD_AILERONS  5 // Aileron
#define ESH_CMD_ELEVATOR  6 // Elevator

/* Effectiveness Matrix definition */
static float G2_T1[ESH_EFF_MAT_COLS_NB]                            = {0}; //scaled by G_SCALE
static float G1_T1[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB]       = {0}; //scaled by G_SCALE
//static float G1G2_T1[ESH_EFF_MAT_ROWS_NB + 1][ESH_EFF_MAT_COLS_NB] = {0}; //scaled by G_SCALE

float ctrl_surfaces_eff = 1;

struct FloatEulers eulers_zxy_ESH;

struct T1_Model T1;

static void eff_scheduling_heewing_update_tilt_angle(void);
static void eff_scheduling_heewing_update_airspeed(void);
static void eff_scheduling_heewing_update_thrust(void);
static void update_attitude(void);
static void sum_copy_EFF_MAT(void);
static void init_T1_Model(void);
static void calc_G1_G2(void);

void eff_scheduling_heewing_init(void)
{
  init_T1_Model();
  update_attitude();
}

void init_T1_Model(void)
{
  // Inertia and mass
  T1.I_XX = ESH_I_XX;
  T1.I_YY = ESH_I_YY;
  T1.I_ZZ = ESH_I_ZZ;
  T1.mass = ESH_MASS;

  T1.chord = ESH_CHORD;
  T1.span = ESH_SPAN;
  T1.aero_coeff = 0.5f * PPRZ_ISA_AIR_DENSITY * T1.chord * T1.span; // 0.5 * density * wing area
  T1.mR.dX = ESH_MR_DX;
  T1.mR.dY = ESH_MR_DY;
  T1.mL.dX = ESH_ML_DX;
  T1.mL.dY = ESH_ML_DY;
  T1.mB.dX = ESH_MB_DX;
  T1.mB.dY = ESH_MB_DY;
}

void update_attitude(void)
{
  float_eulers_of_quat_zxy(&eulers_zxy_ESH, stateGetNedToBodyQuat_f());
  T1.att.phi    = eulers_zxy_ESH.phi;
  T1.att.theta  = eulers_zxy_ESH.theta;
  T1.att.psi    = eulers_zxy_ESH.psi;
  T1.att.sphi   = sinf(eulers_zxy_ESH.phi);
  T1.att.cphi   = cosf(eulers_zxy_ESH.phi);
  T1.att.stheta = sinf(eulers_zxy_ESH.theta);
  T1.att.ctheta = cosf(eulers_zxy_ESH.theta);
  T1.att.spsi   = sinf(eulers_zxy_ESH.psi);
  T1.att.cpsi   = cosf(eulers_zxy_ESH.psi);
}

void eff_scheduling_heewing_periodic(void)
{
  update_attitude();
  eff_scheduling_heewing_update_tilt_angle();
  eff_scheduling_heewing_update_thrust();
  eff_scheduling_heewing_update_airspeed();
  calc_G1_G2();
  sum_copy_EFF_MAT();
}

void eff_scheduling_heewing_update_tilt_angle(void)
{
  // Tilt is normalized between 0. and 99.3deg
  T1.tiltr.rad  = ESH_TILT_ANGLE_MAX * actuator_state_filt_vect[ESH_CMD_TILT_R] / MAX_PPRZ;
  Bound(T1.tiltr.rad, 0.f, ESH_TILT_ANGLE_MAX);
  T1.tiltr.cost  = cosf(T1.tiltr.rad);
  T1.tiltr.sint  = sinf(T1.tiltr.rad);
  T1.tiltl.rad  = ESH_TILT_ANGLE_MAX * actuator_state_filt_vect[ESH_CMD_TILT_L] / MAX_PPRZ;
  Bound(T1.tiltl.rad, 0.f, ESH_TILT_ANGLE_MAX);
  T1.tiltl.cost  = cosf(T1.tiltl.rad);
  T1.tiltl.sint  = sinf(T1.tiltl.rad);
}

void eff_scheduling_heewing_update_thrust(void)
{
  // Thrust is normalized between 0. and 3.5N
  T1.mR.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTOR_R]) / MAX_PPRZ;
  Bound(T1.mR.T, 0.f, ESH_MOTOR_THRUST_MAX);
  T1.mL.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTOR_L]) / MAX_PPRZ;
  Bound(T1.mL.T, 0.f, ESH_MOTOR_THRUST_MAX);
  T1.mB.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTOR_B]) / MAX_PPRZ;
  Bound(T1.mB.T, 0.f, ESH_MOTOR_THRUST_MAX)
}

void eff_scheduling_heewing_update_airspeed(void)
{
  T1.as = stateGetAirspeed_f();
  Bound(T1.as, 0., ESH_AIRSPEED_MAX);
  T1.as2 = T1.as * T1.as;
}

void calc_G1_G2(void)
{
  // Motor Right
  G1_T1[ESH_P][ESH_CMD_MOTOR_R] = -T1.tiltr.sint * (T1.mR.dY / T1.I_XX);
  G1_T1[ESH_Q][ESH_CMD_MOTOR_R] =  T1.tiltr.sint * (T1.mR.dX / T1.I_YY);
  G1_T1[ESH_R][ESH_CMD_MOTOR_R] = -T1.tiltr.cost * (T1.mR.dY / T1.I_ZZ);
  G1_T1[ESH_W][ESH_CMD_MOTOR_R] = -T1.tiltr.sint / T1.mass;
  G1_T1[ESH_U][ESH_CMD_MOTOR_R] =  T1.tiltr.cost / T1.mass;
  G2_T1[ESH_CMD_MOTOR_R]        =  0.f; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Left
  G1_T1[ESH_P][ESH_CMD_MOTOR_L] =  T1.tiltl.sint * (T1.mL.dY / T1.I_XX);
  G1_T1[ESH_Q][ESH_CMD_MOTOR_L] =  T1.tiltl.sint * (T1.mL.dX / T1.I_YY);
  G1_T1[ESH_R][ESH_CMD_MOTOR_L] =  T1.tiltl.cost * (T1.mL.dY / T1.I_ZZ);
  G1_T1[ESH_W][ESH_CMD_MOTOR_L] = -T1.tiltl.sint / T1.mass;
  G1_T1[ESH_U][ESH_CMD_MOTOR_L] =  T1.tiltl.cost / T1.mass;
  G2_T1[ESH_CMD_MOTOR_L]        =  0.f; //T1.mL.dMdud / T1.I_ZZ;
  // Motor Back
  G1_T1[ESH_Q][ESH_CMD_MOTOR_B] = -T1.mB.dX / T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTOR_B] = -1.f / T1.mass;
  G2_T1[ESH_CMD_MOTOR_B]        =  0.f; //-T1.mB.dMdud / T1.I_ZZ;
  // Tilt Right
  G1_T1[ESH_P][ESH_CMD_TILT_R]  = -T1.mR.T * T1.tiltr.cost * (T1.mR.dY / T1.I_XX);
  G1_T1[ESH_Q][ESH_CMD_TILT_R]  =  T1.mR.T * T1.tiltr.cost * (T1.mR.dX / T1.I_YY);
  G1_T1[ESH_R][ESH_CMD_TILT_R]  = -T1.mR.T * T1.tiltr.sint * (T1.mR.dY / T1.I_ZZ);
  G1_T1[ESH_W][ESH_CMD_TILT_R]  = -T1.mR.T * T1.tiltr.cost / T1.mass;
  G1_T1[ESH_U][ESH_CMD_TILT_R]  =  T1.mR.T * T1.tiltr.sint / T1.mass;
  // Tilt Left
  G1_T1[ESH_P][ESH_CMD_TILT_L]  =  T1.mL.T * T1.tiltl.cost * (T1.mL.dY / T1.I_XX);
  G1_T1[ESH_P][ESH_CMD_TILT_L]  =  T1.mL.T * T1.tiltl.cost * (T1.mL.dX / T1.I_YY);
  G1_T1[ESH_P][ESH_CMD_TILT_L]  =  T1.mL.T * T1.tiltl.sint * (T1.mL.dY / T1.I_ZZ);
  G1_T1[ESH_W][ESH_CMD_TILT_L]  = -T1.mL.T * T1.tiltl.cost / T1.mass;
  G1_T1[ESH_U][ESH_CMD_TILT_L]  =  T1.mL.T * T1.tiltl.sint / T1.mass;
  // Criteria to not saturate control surfaces at low speed and start using them at around 5 m/s
  //if (T1.as < 5) {
  //  ctrl_surfaces_eff = 0; //-9.8 * T1.as + 50; // VERIFY IF IT WORKS AT VERY LOW SPEEDS (near 0)
  //} else {
  ctrl_surfaces_eff = 1;
  //}
  // Aileron
  G1_T1[ESH_P][ESH_CMD_AILERONS] = ctrl_surfaces_eff * T1.aero_coeff * T1.as2 * T1.span * ESH_ROLL_EFF / T1.I_XX;
  // Elevator
  G1_T1[ESH_Q][ESH_CMD_ELEVATOR] = ctrl_surfaces_eff * T1.aero_coeff * T1.as2 * T1.chord * ESH_PITCH_EFF / T1.I_YY;
}

void sum_copy_EFF_MAT(void)
{
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
      g1g2[i][j] = G1_T1[i][j] / INDI_G_SCALING; // FIXME is it necessary to scale ?
    }
  }
}

void stabilization_indi_set_wls_settings(void)
{
  float mean_tilt = (actuator_state_filt_vect[ESH_CMD_TILT_R] + actuator_state_filt_vect[ESH_CMD_TILT_R]) / 2.f;

  for (int8_t i = 0; i < ESH_EFF_MAT_COLS_NB; i++) {
    switch (i) {
      case (ESH_CMD_MOTOR_R):
      case (ESH_CMD_MOTOR_L):
      case (ESH_CMD_MOTOR_B):
        wls_stab_p.u_min[i] = ESH_MOTOR_IDLE;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case (ESH_CMD_TILT_R):
        if (autopilot_get_mode() == AP_MODE_NAV) {
          wls_stab_p.u_min[i] = Max(mean_tilt - ESH_TILT_DIFF_MAX/2, 0);
          wls_stab_p.u_max[i] = Min(mean_tilt - ESH_TILT_DIFF_MAX/2, MAX_PPRZ);
          wls_stab_p.u_pref[i] = actuator_state_filt_vect[ESH_CMD_TILT_R];
        } else if (autopilot_get_mode() == AP_MODE_FORWARD) {
          wls_stab_p.u_min[i] = ESH_TILT_FORWARD; // Max(mean_tilt - ESH_TILT_DIFF_MAX/2, 0);
          wls_stab_p.u_max[i] = ESH_TILT_FORWARD; // Min(mean_tilt - ESH_TILT_DIFF_MAX/2, MAX_PPRZ);
          wls_stab_p.u_pref[i] = ESH_TILT_FORWARD;
        } else {
          // default: tilt in hover position
          wls_stab_p.u_min[i] = ESH_TILT_VERTICAL;
          wls_stab_p.u_max[i] = ESH_TILT_VERTICAL;
          wls_stab_p.u_pref[i] = ESH_TILT_VERTICAL;
        }
        break;
      case (ESH_CMD_TILT_L):
        if (autopilot_get_mode() == AP_MODE_NAV) {
          wls_stab_p.u_min[i] = Max(mean_tilt - ESH_TILT_DIFF_MAX/2, 0);
          wls_stab_p.u_max[i] = Min(mean_tilt - ESH_TILT_DIFF_MAX/2, MAX_PPRZ);
          wls_stab_p.u_pref[i] = actuator_state_filt_vect[ESH_CMD_TILT_L];
        } else if (autopilot_get_mode() == AP_MODE_FORWARD) {
          wls_stab_p.u_min[i] = ESH_TILT_FORWARD; // Max(mean_tilt - ESH_TILT_DIFF_MAX/2, 0);
          wls_stab_p.u_max[i] = ESH_TILT_FORWARD; // Min(mean_tilt - ESH_TILT_DIFF_MAX/2, MAX_PPRZ);
          wls_stab_p.u_pref[i] = ESH_TILT_FORWARD;
        } else {
          // default: tilt in hover position
          wls_stab_p.u_min[i] = ESH_TILT_VERTICAL;
          wls_stab_p.u_max[i] = ESH_TILT_VERTICAL;
          wls_stab_p.u_pref[i] = ESH_TILT_VERTICAL;
        }
        break;
      case (ESH_CMD_AILERONS):
      case (ESH_CMD_ELEVATOR):
        wls_stab_p.u_min[i] = -MAX_PPRZ;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      default:
        break;
    }
  }
}

// Override standard LIFT_D function
float guidance_indi_get_liftd(float airspeed, float theta UNUSED)
{
  return (-T1.aero_coeff * airspeed * airspeed * ESH_LIFT_EFF);
}

float guidance_indi_max_pusher_incr(void)
{
  return MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTOR_R] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTOR_L] +
         MAX_PPRZ * g1g2[ESH_U][ESH_CMD_TILT_R] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_TILT_L];
}

//void update_total_thrust(int32_t *cmd)
//{
//  cmd[COMMAND_THRUST] = (
//      actuator_state_filt_vect[ESH_CMD_MOTOR_R] +
//      actuator_state_filt_vect[ESH_CMD_MOTOR_L] +
//      actuator_state_filt_vect[ESH_CMD_MOTOR_B]) / 3;
//}
