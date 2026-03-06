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

// Motor idle position
#ifndef ESH_MOTOR_IDLE
#define ESH_MOTOR_IDLE 800
#endif

// Max tilt diff
#ifndef ESH_TILT_DIFF_MAX
#define ESH_TILT_DIFF_MAX 900
#endif

// Max tilt angle
#ifndef ESH_TILT_ANGLE_MAX
#define ESH_TILT_ANGLE_MAX RadOfDeg(99.3f)
#endif

// Max motor thrust in Newtons
#ifndef ESH_MOTOR_THRUST_MAX
#define ESH_MOTOR_THRUST_MAX 3.5f
#endif

// Max airspeed (bound airspeed data) in m/s
#ifndef ESH_AIRSPEED_MAX
#define ESH_AIRSPEED_MAX 30.f
#endif

// WLS Tilt Diff Weight
#ifndef ESH_WLS_MIN_MT
#define ESH_WLS_MIN_MT 0.0f
#endif

// Default mechanical characteristics
// based on T1 Ranger

#ifndef ESH_CHORD
#define ESH_CHORD 0.15f
#endif

#ifndef ESH_SPAN
#define ESH_SPAN 0.73f
#endif

#define ESH_MR_DX
#define ESH_MR_DX 0.09f
#endif

#define ESH_MR_DY
#define ESH_MR_DY 0.12f
#endif

#define ESH_ML_DX
#define ESH_ML_DX 0.09f
#endif

#define ESH_ML_DY
#define ESH_ML_DY 0.12f
#endif

#define ESH_MB_DX
#define ESH_MB_DX 0.25f
#endif

#define ESH_MB_DY
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

#define ESH_CMD_MOTORR    0 // Motor Right
#define ESH_CMD_MOTORL    1 // Motor Left
#define ESH_CMD_MOTORB    2 // Motor Back
#define ESH_CMD_MOTORMT   3 // Motor Mean Tilt
#define ESH_CMD_MOTORTD   4 // Motor Tilt Diff
#define ESH_CMD_AILERONS  5 // Aileron
#define ESH_CMD_ELEVATOR  6 // Elevator

/* Effectiveness Matrix definition */
static float G2_T1[ESH_EFF_MAT_COLS_NB]                            = {0}; //scaled by G_SCALE
static float G1_T1[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB]       = {0}; //scaled by G_SCALE
static float G1G2_T1[ESH_EFF_MAT_ROWS_NB + 1][ESH_EFF_MAT_COLS_NB] = {0}; //scaled by G_SCALE

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
  T1.wls_min_mt = ESH_WLS_MIN_MT;

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
  T1.tiltr.rad  = ESH_TILT_ANGLE_MAX * (actuator_state_filt_vect[ESH_CMD_MOTORMT] + actuator_state_filt_vect[ESH_CMD_MOTORTD]) / MAX_PPRZ;
  Bound(T1.tiltr.rad, 0.f, ESH_TILT_ANGLE_MAX);
  T1.tiltr.cosr  = cosf(T1.tiltr.rad);
  T1.tiltr.sinr  = sinf(T1.tiltr.rad);
  T1.tiltl.rad  = ESH_TILT_ANGLE_MAX * (actuator_state_filt_vect[ESH_CMD_MOTORMT] - actuator_state_filt_vect[ESH_CMD_MOTORTD]) / MAX_PPRZ;
  Bound(T1.tiltl.rad, 0.f, ESH_TILT_ANGLE_MAX);
  T1.tiltl.cosr  = cosf(T1.tiltl.rad);
  T1.tiltl.sinr  = sinf(T1.tiltl.rad);
}

void eff_scheduling_heewing_update_thrust(void)
{
  // Thrust is normalized between 0. and 3.5N
  T1.mR.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTORR]) / MAX_PPRZ;
  Bound(T1.mR.T, 0.f, ESH_MOTOR_THRUST_MAX);
  T1.mL.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTORL]) / MAX_PPRZ;
  Bound(T1.mL.T, 0.f, ESH_MOTOR_THRUST_MAX);
  T1.mB.T = ESH_MOTOR_THRUST_MAX * (actuator_state_filt_vect[ESH_CMD_MOTORB]) / MAX_PPRZ;
  Bound(T1.mB.T, 0.f, ESH_MOTOR_THRUST_MAX)
}

void eff_scheduling_heewing_update_airspeed(void)
{
  T1.as = stateGetAirspeed_f();
  Bound(T1.as, 0., ESH_AIRSPEED_MAX);
  T1.as2 = T1.as * T1.as;
  Bound(T1.as2, 0., ESH_AIRSPEED_MAX * ESH_AIRSPEED_MAX);
}

void calc_G1_G2(void)
{
  // Motor Right
  G1_T1[ESH_P][ESH_CMD_MOTORR]  = -0.444444 * (T1.tiltr.sinr * T1.mR.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORR]  =  0.444444 * (T1.tiltr.sinr * T1.mR.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORR]  = -0.444444 * (T1.tiltr.cosr * T1.mR.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORR]  = -0.567724 * T1.tiltr.sinr / T1.mass;
  G1_T1[ESH_U][ESH_CMD_MOTORR]  =  0.567724 * T1.tiltr.cosr / T1.mass;
  G2_T1[ESH_CMD_MOTORR]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Left
  G1_T1[ESH_P][ESH_CMD_MOTORL]  =  0.444444 * (T1.tiltl.sinr * T1.mL.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORL]  =  0.444444 * (T1.tiltl.sinr * T1.mL.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORL]  =  0.444444 * (T1.tiltl.cosr * T1.mL.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORL]  = -0.567724 * T1.tiltl.sinr / T1.mass;
  G1_T1[ESH_U][ESH_CMD_MOTORL]  =  0.567724 * T1.tiltl.cosr / T1.mass;
  G2_T1[ESH_CMD_MOTORL]         =  0; //T1.mL.dMdud / T1.I_ZZ;
  // Motor Back
  G1_T1[ESH_Q][ESH_CMD_MOTORB]  = -0.444444 * T1.mB.dX / T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTORB]  = -0.567724 / T1.mass;
  G2_T1[ESH_CMD_MOTORB]         =  0; //-T1.mB.dMdud / T1.I_ZZ;
  // Motor Mean Tilt
  G1_T1[ESH_Q][ESH_CMD_MOTORMT]  = 0.2 * (T1.mR.T * T1.tiltr.sinr * T1.mR.dX + T1.mL.T * T1.tiltl.sinr * T1.mL.dX) /
                                   T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTORMT]  = -0.0003 * (T1.mR.T * T1.tiltr.cosr + T1.mL.T * T1.tiltl.cosr) / T1.mass;
  G1_T1[ESH_U][ESH_CMD_MOTORMT]  = -0.0003 * (T1.mR.T * T1.tiltr.sinr + T1.mL.T * T1.tiltl.sinr) / T1.mass;
  G2_T1[ESH_CMD_MOTORMT]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Tilt Diff
  G1_T1[ESH_P][ESH_CMD_MOTORTD]  = -1.536098 * (T1.mR.T * T1.tiltr.cosr * T1.mR.dY + T1.mL.T * T1.tiltl.cosr *
                                   T1.mL.dY) / (2 * T1.I_ZZ);
  G1_T1[ESH_R][ESH_CMD_MOTORTD]  =  1.536098 * (T1.mR.T * T1.tiltr.sinr * T1.mR.dY + T1.mL.T * T1.tiltl.sinr *
                                    T1.mL.dY) / (2 * T1.I_ZZ);
  G2_T1[ESH_CMD_MOTORTD]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Criteria to not saturate control surfaces at low speed and start using them at around 5 m/s
  //if (T1.as < 5) {
  //  ctrl_surfaces_eff = 0; //-9.8 * T1.as + 50; // VERIFY IF IT WORKS AT VERY LOW SPEEDS (near 0)
  //} else {
  ctrl_surfaces_eff = 1;
  //}
  // Aileron
  G1_T1[ESH_P][ESH_CMD_AILERONS] = 1.0E-6 * ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.span * ROLL_D2_AILERONS) /
                                   T1.I_XX;
// Elevator
  G1_T1[ESH_Q][ESH_CMD_ELEVATOR] = 1.0E-6 * ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.chord * PITCH_D2_ELEVATOR) /
                                   T1.I_YY;
}

void sum_copy_EFF_MAT(void)
{
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
      g1g2[i][j] = G1_T1[i][j] / INDI_G_SCALING;
    }
  }
}

void stabilization_indi_set_wls_settings(void)
{
  for (int8_t i = 0; i < ESH_EFF_MAT_COLS_NB; i++) {
    switch (i) {
      case (ESH_CMD_MOTORR):
      case (ESH_CMD_MOTORL):
      case (ESH_CMD_MOTORB):
        wls_stab_p.u_min[i] = ESH_MOTOR_IDLE;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case (ESH_CMD_MOTORTD):
        wls_stab_p.u_min[i] = -ESH_TILT_DIFF_MAX;
        wls_stab_p.u_max[i] = ESH_TILT_DIFF_MAX;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case (ESH_CMD_MOTORMT):
        wls_stab_p.u_min[i] = T1.wls_min_mt;
        wls_stab_p.u_max[i] = MAX_PPRZ - ESH_TILT_DIFF_MAX;
        wls_stab_p.u_pref[i] = act_pref[i];
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
  return (-T1.aero_coeff * airspeed * airspeed * LIFT_D2_ALPHA);
}

float get_max_pusher_thrust(void)
{
  return MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORR] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORL] +
         MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORMT] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORTD];
}

void update_total_thrust(int32_t *cmd)
{
  cmd[COMMAND_THRUST]   = (actuator_state_filt_vect[ESH_CMD_MOTORR] + actuator_state_filt_vect[ESH_CMD_MOTORL] +
                           actuator_state_filt_vect[ESH_CMD_MOTORB]) / 3;
}
