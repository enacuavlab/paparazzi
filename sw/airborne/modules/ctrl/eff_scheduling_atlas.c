/*
 * Copyright (C) 2025 Mauro VA <mauro.villanueva-aguado@enac.fr>
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

/** @file "modules/ctrl/eff_scheduling_atlas.c"
 * Control effectiveness scheduler for the Atlas tiltrotor with
 * two tilt banks (right/left) each carrying 2 rotor in a quad-X layout
 *
 * Physical model with motor at body position r = (x, y, z) with tilt angle a and thrust T:
 *
 * Mx = -y * T * cos(a) + κ * spin * T * sin(a)
 * My = z * T * sin(a) + x * T * cos(a)
 * Mz = -y * T * sin(a) - κ * spin * T * cos(a)
 * Fx = T * sin(a)
 * Fz = -T * cos(a)
 *
*/

#include "modules/ctrl/eff_scheduling_atlas.h"

#include "generated/airframe.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"

#include "modules/actuators/actuators.h"
#include "math.h"

// Airframe file parameter checks
#ifndef ATLAS_EFF_IXX
#error "ATLAS_EFF_IXX [float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_IYY
#error "ATLAS_EFF_IYY [float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_IZZ
#error "ATLAS_EFF_IZZ [float] not defined in airframe file"
#endif
#ifndef GUIDANCE_INDI_MASS
#error "GUIDANCE_INDI_MASS [float] not defined in airframe file"
#endif

#ifndef ATLAS_EFF_RX
#error "ATLAS_EFF_RX [4x1 float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_RY
#error "ATLAS_EFF_RY [4x1 float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_RZ
#error "ATLAS_EFF_RZ [4x1 float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_K_DT_DPPRZ
#error "ATLAS_EFF_K_DT_DPPRZ [2x1 float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_KAPPA
#error "ATLAS_EFF_KAPPA [float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_SPIN_DIR
#error "ATLAS_EFF_SPIN_DIR [4x1 float] (+/-1) not defined in airframe file"
#endif
#ifndef ATLAS_EFF_K_TILT_DEFLECT
#error "ATLAS_EFF_K_TILT_DEFLECT [2x1 float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_K_LIFT
#error "ATLAS_EFF_K_LIFT [float] not defined in airframe file"
#endif
#ifndef ATLAS_EFF_V_WING
#define ATLAS_EFF_V_WING 10.0f
#endif

// #ifndef ATLAS_EFF_K_ELEVON_DEFLECT
// #error "ATLAS_EFF_K_ELEVON_DEFLECT [2x1 float] not defined in airframe file"
// #endif
// #ifndef ATLAS_EFF_K_ELEVON_ROLL
// #error "ATLAS_EFF_K_ELEVON_ROLL [float] not defined in airframe file"
// #endif
// #ifndef ATLAS_EFF_K_ELEVON_PITCH
// #error "ATLAS_EFF_K_ELEVON_PITCH [float] not defined in airframe file"
// #endif
// #ifndef ATLAS_EFF_K_ELEVON_PROPWASH
// #define ATLAS_EFF_K_ELEVON_PROPWASH 0.0f
// #endif

// Motor Hover Value [pprz]
#ifndef ATLAS_MOTOR_HOVER
#error "ATLAS_MOTOR_HOVER [float] not defined in airframe file"
#endif
#ifndef ATLAS_MOTOR_IDLE
#error "ATLAS_MOTOR_IDLE [float] not defined in airframe file"
#endif

// Tilt Range
#ifndef ATLAS_TILT_ALPHA_MIN
#define ATLAS_TILT_ALPHA_MIN  -0.1f
#endif
#ifndef ATLAS_TILT_ALPHA_MAX
#define ATLAS_TILT_ALPHA_MAX  (0.5f * M_PI)
#endif


struct atlas_eff_sched_param_t atlas_eff_sched_p = {
 .Ixx = ATLAS_EFF_IXX,
 .Iyy = ATLAS_EFF_IYY,
 .Izz = ATLAS_EFF_IZZ,
 .m = GUIDANCE_INDI_MASS,
 .r_x = ATLAS_EFF_RX,
 .r_y = ATLAS_EFF_RY,
 .r_z = ATLAS_EFF_RZ,
 .k_dT_dpprz = ATLAS_EFF_K_DT_DPPRZ,
 .kappa = ATLAS_EFF_KAPPA,
 .spin_dir = ATLAS_EFF_SPIN_DIR,
 .k_tilt_deflect = ATLAS_EFF_K_TILT_DEFLECT,
 .k_lift = ATLAS_EFF_K_LIFT,
 .v_wing = ATLAS_EFF_V_WING,
 // .k_elevon_deflect = ATLAS_EFF_K_ELEVON_DEFLECT,
 // .k_elevon_roll = ATLAS_EFF_K_ELEVON_ROLL,
 // .k_elevon_pitch = ATLAS_EFF_K_ELEVON_PITCH,
 // .k_elevon_propwash = ATLAS_EFF_K_ELEVON_PROPWASH,
};

struct atlas_eff_sched_var_t atlas_eff_sched_v;

float const grav = 9.81f;                           // Gravitational Acceleration [m/s^2]
float atlas_eff_periodic_freq = EFF_SCHEDULING_ATLAS_PERIODIC_FREQ; // Module Frequency [Hz]

float atlas_eff_liftd = 0.0f;                 // Change in Lift wrt change in pitch (dLift/dpitch) [N/rad]
float atlas_eff_tilt_traverse_time = 1.5f;    // Time to traverse tilt range [s]

/** Helper Function:
 * Bounds or zeros values to improve WLS optimization
 * @param value  Input to be processed
 * @param low    Lower bound of interval
 * @param up     Upper bound of interval
 *
 * @return clamped within bounds and zeroed if outside active side of interval
 */
static inline float bound_or_zero(float value, float low, float up)
{
 if (low > 0.f)
 {
  if (value < low) return 0.f;
  if (value > up) return up;
 }
 else
 {
  if (value < low) return low;
  if (value > up) return 0.f;
 }
 return value;
}

// Function declarations
static inline void atlas_update_tilt_cache(void);
static inline void atlas_update_cmd_cache(void);
static inline void atlas_update_airspeed_cache(void);
static inline void atlas_update_thrust_model(void);
static inline void atlas_update_motor_effectiveness(void);
static inline void atlas_update_tilt_effectiveness(void);
// static inline void atlas_update_elevon_effectiveness(void);
static inline void atlas_schedule_liftd(void);

float guidance_indi_get_liftd(float airspeed UNUSED, float theta UNUSED);
void stabilization_indi_set_wls_settings(void);
void eff_scheduling_atlas_init(void)
{
  // Checks for positive and non-zeros coefficients
  const float a1 = atlas_eff_sched_p.k_dT_dpprz[0];
  const float b1 = atlas_eff_sched_p.k_tilt_deflect[1];

  if (a1 < 0.f || b1 == 0)
  {
  /*
   * Prevent the stabilization module from stating if (1) a1 is negative or (2) b1 is zero:
   * (1):
   * The change in thrust due to change in pprz is given by: dT/du = a1 + 2*a2*u,
   * if a1 is negative an increase in pprz results in a decrease in thrust breaking the thrust model
   * (2):
   * The change in tilt angle due to a change in pprz is given by: alpha = b0 + b1 * pprz
   * if b1 is zero the tilt cannot be commanded
  */
    while (1) { /* trap */ }
  }

  // Tilt angles initialization
  atlas_eff_sched_v.alpha_r_rad = 0.f;
  atlas_eff_sched_v.alpha_l_rad = 0.f;
  atlas_eff_sched_v.cos_ar = 1.f; atlas_eff_sched_v.cos_al = 1.f;
  atlas_eff_sched_v.sin_ar = 0.f; atlas_eff_sched_v.sin_al = 0.f;

  for (int i = 0; i < 4; i++)
  {
  atlas_eff_sched_v.cmd_motor[i] = ATLAS_MOTOR_HOVER;
  atlas_eff_sched_v.T[i] = atlas_eff_sched_p.m*grav/4.f;
  atlas_eff_sched_v.dT_dpprz[i] = atlas_eff_sched_p.k_dT_dpprz[0];
  }
  atlas_eff_sched_v.airspeed = 0.f;
  atlas_eff_sched_v.airspeed_sq = 0.f;

  // atlas_eff_sched_v.cmd_elevon_r = 0.f;
  // atlas_eff_sched_v.cmd_elevon_l = 0.f;
}

void eff_scheduling_atlas_periodic(void)
{
  atlas_update_tilt_cache();
  atlas_update_cmd_cache();
  atlas_update_airspeed_cache();
  atlas_update_thrust_model();

  atlas_update_motor_effectiveness();
  atlas_update_tilt_effectiveness();
  // atlas_update_elevon_effectiveness();
  atlas_schedule_liftd();
}

static inline void atlas_update_tilt_cache(void)
{
  const float c0 = atlas_eff_sched_p.k_tilt_deflect[0];
  const float c1 = atlas_eff_sched_p.k_tilt_deflect[1];

  atlas_eff_sched_v.alpha_r_rad = c0 + c1 * actuator_state_filt_vect[ATLAS_ACT_TILT_R];
  atlas_eff_sched_v.alpha_l_rad = c0 + c1 * actuator_state_filt_vect[ATLAS_ACT_TILT_L];

  /* Enforce tilt range bound for limits  */
  Bound(atlas_eff_sched_v.alpha_r_rad, ATLAS_TILT_ALPHA_MIN, ATLAS_TILT_ALPHA_MAX);
  Bound(atlas_eff_sched_v.alpha_l_rad, ATLAS_TILT_ALPHA_MIN, ATLAS_TILT_ALPHA_MAX);

  atlas_eff_sched_v.cos_ar = cosf(atlas_eff_sched_v.alpha_r_rad);
  atlas_eff_sched_v.sin_ar = sinf(atlas_eff_sched_v.alpha_r_rad);
  atlas_eff_sched_v.cos_al = cosf(atlas_eff_sched_v.alpha_l_rad);
  atlas_eff_sched_v.sin_al = sinf(atlas_eff_sched_v.alpha_l_rad);
}

static inline void atlas_update_cmd_cache(void)
{
  atlas_eff_sched_v.cmd_motor[0] = actuator_state_filt_vect[ATLAS_ACT_MOTOR_FR];
  atlas_eff_sched_v.cmd_motor[1] = actuator_state_filt_vect[ATLAS_ACT_MOTOR_BR];
  atlas_eff_sched_v.cmd_motor[2] = actuator_state_filt_vect[ATLAS_ACT_MOTOR_BL];
  atlas_eff_sched_v.cmd_motor[3] = actuator_state_filt_vect[ATLAS_ACT_MOTOR_FL];
  // atlas_eff_sched_v.cmd_elevon_r = actuator_state_filt_vect[ATLAS_ACT_ELEVON_R];
  // atlas_eff_sched_v.cmd_elevon_l = actuator_state_filt_vect[ATLAS_ACT_ELEVON_L];
}

static inline void atlas_update_airspeed_cache(void)
{
  atlas_eff_sched_v.airspeed = stateGetAirspeed_f();
  Bound(atlas_eff_sched_v.airspeed, 0.f, 30.f);
  atlas_eff_sched_v.airspeed_sq = atlas_eff_sched_v.airspeed * atlas_eff_sched_v.airspeed;
}

/* Quadratic thrust model:
 *   T(u)      = a + b*u + c*u^2
 *   dT/du(u)  = b + 2*c*u
 * k_dT_dpprz = [b, c].
 */
static inline void atlas_update_thrust_model(void)
{
  const float b = atlas_eff_sched_p.k_dT_dpprz[0];
  const float c = atlas_eff_sched_p.k_dT_dpprz[1];
  for (int i = 0; i < 4; i++) {
    float u = atlas_eff_sched_v.cmd_motor[i];
    Bound(u, 0.f, MAX_PPRZ);
    atlas_eff_sched_v.T[i]       = (b + c * u) * u;
    atlas_eff_sched_v.dT_dpprz[i] = b + 2.f * c * u;
    Bound(atlas_eff_sched_v.dT_dpprz[i], 0.1f * b, 3.0f * b);
  }
}

/*
 * Control effectiveness wrt. motor throttle (dT = dT/du)
 * dMx/du = -y * dT * cos(a) + spin * κ * dT * sin(a)
 * dMy/du = x * dT * cos(a) + z * dT * sin(a)
 * dMz/du = -y * dT * sin(a) - spin * κ * dT * cos(a)
 * dFx/du = dT * sin(a)
 * dFz/du = -dT * cos(a)
 */
static inline void atlas_update_motor_effectiveness(void)
{
  for (int i = 0; i < 4; i++) {
    const float x  = atlas_eff_sched_p.r_x[i];
    const float y  = atlas_eff_sched_p.r_y[i];
    const float z  = atlas_eff_sched_p.r_z[i];
    const float spin = atlas_eff_sched_p.spin_dir[i];
    const float kappa = atlas_eff_sched_p.kappa;
    const float dT = atlas_eff_sched_v.dT_dpprz[i];

    const int right_bank = (i == 0 || i == 1);
    const float ca = right_bank ? atlas_eff_sched_v.cos_ar : atlas_eff_sched_v.cos_al;
    const float sa = right_bank ? atlas_eff_sched_v.sin_ar : atlas_eff_sched_v.sin_al;

    /* Angular-acceleration effectiveness wrt. motor commands */
    float dMx = (-y * dT * ca + spin * kappa * dT * sa);
    float dMy = ( x * dT * ca + z * dT * sa);
    float dMz = (-y * dT * sa - spin * kappa * dT * ca);

    /* Linear-acceleration effectiveness wrt. motor commands*/
    float dAx =  dT * sa;
    float dAz = -dT * ca;

    g1g2[ATLAS_VC_MX][i] = dMx / atlas_eff_sched_p.Ixx;
    g1g2[ATLAS_VC_MY][i] = dMy / atlas_eff_sched_p.Iyy;
    g1g2[ATLAS_VC_MZ][i] = dMz / atlas_eff_sched_p.Izz;
    g1g2[ATLAS_VC_AX][i] = dAx / atlas_eff_sched_p.m;
    g1g2[ATLAS_VC_AZ][i] = dAz / atlas_eff_sched_p.m;
  }
}

/*
 * Control effectiveness wrt. tilt angle (da = da/du)
 * dMx/du = (y * T * sin(a) + spin * κ * T * cos(a)) * da
 * dMy/du = (z * T * cos(a) - x * T * sin(a)) * da
 * dMz/du - (-y * T * cos(a) + spin * κ * T * sin(a)) * da
 * dFx/du = T * cos(a) * da
 * dFz/du = T * sin(a) * da
 */
static inline void atlas_update_tilt_effectiveness(void)
{
  const float da = atlas_eff_sched_p.k_tilt_deflect[1]; /* rad per pprz */
  const float kappa   = atlas_eff_sched_p.kappa;

  for (int b = 0; b < 2; b++) {
    const int   idx = (b == 0) ? ATLAS_ACT_TILT_R : ATLAS_ACT_TILT_L;
    const float ca = (b == 0) ? atlas_eff_sched_v.cos_ar : atlas_eff_sched_v.cos_al;
    const float sa = (b == 0) ? atlas_eff_sched_v.sin_ar : atlas_eff_sched_v.sin_al;
    const int   motor_a = (b == 0) ? 0 : 2; // (FR (0) / BL (2))
    const int   motor_b = (b == 0) ? 1 : 3; // (BR (1) / FL (3))

    float dMx = 0.f, dMy = 0.f, dMz = 0.f, dAx = 0.f, dAz = 0.f;

    const int m[2] = { motor_a, motor_b };
    for (int k = 0; k < 2; k++) {
      const int   i  = m[k];
      const float x  = atlas_eff_sched_p.r_x[i];
      const float y  = atlas_eff_sched_p.r_y[i];
      const float z  = atlas_eff_sched_p.r_z[i];
      const float spin = atlas_eff_sched_p.spin_dir[i];
      const float T  = atlas_eff_sched_v.T[i];

      dMx += (y * T * sa + spin * kappa * T * ca) * da;
      dMy += (z * T * ca - x  * T * sa) * da;
      dMz += (-y * T * ca + spin * kappa * T * sa) * da;
      dAx +=  T * ca * da;
      dAz +=  T * sa * da;
    }

    g1g2[ATLAS_VC_MX][idx] = dMx  / atlas_eff_sched_p.Ixx;
    g1g2[ATLAS_VC_MY][idx] = dMy  / atlas_eff_sched_p.Iyy;
    g1g2[ATLAS_VC_MZ][idx] = dMz  / atlas_eff_sched_p.Izz;
    g1g2[ATLAS_VC_AX][idx] = dAx / atlas_eff_sched_p.m;
    g1g2[ATLAS_VC_AZ][idx] = dAz / atlas_eff_sched_p.m;
  }
}

// /*
//  * Control Effectiveness wrt. elevons
//  */
// static inline void atlas_update_elevon_effectiveness(void)
// {
//   float Tx = atlas_eff_sched_v.T[0] * atlas_eff_sched_v.sin_ar
//             + atlas_eff_sched_v.T[1] * atlas_eff_sched_v.sin_ar
//             + atlas_eff_sched_v.T[2] * atlas_eff_sched_v.sin_al
//             + atlas_eff_sched_v.T[3] * atlas_eff_sched_v.sin_al;
//
//   const float dDelta = atlas_eff_sched_p.k_elevon_deflect[1]; // [rad per pprz]
//
//   // dM/dDelta (assumes symmetric tilt left/right
//   float dMx_dDelta = atlas_eff_sched_p.k_elevon_roll * atlas_eff_sched_v.airspeed_sq;
//   float dMy_dDelta = atlas_eff_sched_p.k_elevon_pitch * atlas_eff_sched_v.airspeed_sq
//                     + atlas_eff_sched_p.k_elevon_propwash * Tx * atlas_eff_sched_v.airspeed;
//
//   float dMx = dMx_dDelta * dDelta / atlas_eff_sched_p.Ixx;
//   float dMy = dMy_dDelta * dDelta / atlas_eff_sched_p.Iyy;
//
//   Bound(dMx, 0.f, 0.1f);
//   Bound(dMy, 0.f, 0.1f);
//
//   // Elevon effectiveness matrix
//   // Right Elevon (pitch up -> +delta, roll right -> +delta)
//   g1g2[ATLAS_VC_MX][ATLAS_ACT_ELEVON_R] = dMx;
//   g1g2[ATLAS_VC_MY][ATLAS_ACT_ELEVON_R] = dMy;
//   // Left Elevon (pitch up -> +delta, roll right -> -delta)
//   g1g2[ATLAS_VC_MX][ATLAS_ACT_ELEVON_L] = -dMx;
//   g1g2[ATLAS_VC_MY][ATLAS_ACT_ELEVON_L] = dMy;
// }


// Inner Loop Stabilization WLS settings
void stabilization_indi_set_wls_settings(void)
{
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    wls_stab_p.u_min[i] = act_is_servo[i] ? -MAX_PPRZ : ATLAS_MOTOR_IDLE;
    wls_stab_p.u_max[i]  =  MAX_PPRZ;
    wls_stab_p.u_pref[i] =  act_pref[i];
  }

  // Convert Tilt Range rad to pprz units using:
  // alpha [rad] = k_tilt_deflect[0] + k_tilt_deflect[1] * pprz [pprz]
  // (if increasing pprz -> decreases alpha swap min/max ranges)
  const float c0 = atlas_eff_sched_p.k_tilt_deflect[0];
  const float c1 = atlas_eff_sched_p.k_tilt_deflect[1];
  if (c1 != 0.f) {
    const float pprz_a = (ATLAS_TILT_ALPHA_MIN - c0) / c1;
    const float pprz_b = (ATLAS_TILT_ALPHA_MAX - c0) / c1;
    const float tilt_pprz_min = (pprz_a < pprz_b) ? pprz_a : pprz_b;
    const float tilt_pprz_max = (pprz_a < pprz_b) ? pprz_b : pprz_a;

    // Tilt servos (rate-limit and range-limit)
    // Rate-limit per step
    Bound(atlas_eff_tilt_traverse_time, 0.002f, 5.0f);
    const float tilt_incr_max = (tilt_pprz_max - tilt_pprz_min) / (atlas_eff_tilt_traverse_time * atlas_eff_periodic_freq);

    for (int s = ATLAS_ACT_TILT_R; s <= ATLAS_ACT_TILT_L; s++) {
      float u_min = actuators_pprz[s] - tilt_incr_max;
      float u_max = actuators_pprz[s] + tilt_incr_max;

      /* Intersect with absolute mechanical range */
      if (u_min < tilt_pprz_min) u_min = tilt_pprz_min;
      if (u_max > tilt_pprz_max) u_max = tilt_pprz_max;

      /* Clip to ensure within servo limits */
      Bound(u_min, -MAX_PPRZ, MAX_PPRZ);
      Bound(u_max, -MAX_PPRZ, MAX_PPRZ);

      wls_stab_p.u_min[s] = u_min;
      wls_stab_p.u_max[s] = u_max;
      }
    }

  // // Elevons
  // for (int e = TW_ACT_ELEVON_R; e <= TW_ACT_ELEVON_L; e++) {
  //   wls_stab_p.u_pref[e] = actuator_state_filt_vect[e];
  // }
}

/*
 * Wing lift derivative wrt. pitch
 * @return atlas_eff_liftd: lift derivative
 */
static inline void atlas_schedule_liftd(void)
{
  float lift_d = atlas_eff_sched_p.k_lift * atlas_eff_sched_v.airspeed_sq / atlas_eff_sched_p.m;

  // Apply lift derivative contribution above a certian speed (v_wind)
  if (atlas_eff_sched_v.airspeed < atlas_eff_sched_p.v_wing) {
    lift_d = 0.f;
  }
  Bound(lift_d, -150.f, 0.f);
  atlas_eff_liftd = lift_d;
}

float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
  return atlas_eff_liftd;
}
