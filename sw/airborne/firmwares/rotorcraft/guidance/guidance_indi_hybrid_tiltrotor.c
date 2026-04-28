/*
* Copyright (C) 2026 Mauro VA <mauro.villanueva-aguado@enac.fr>
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
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"                         // for stab_thrust_filt
#include "math/pprz_isa.h"                                                                 // Air density Constant

// Input and output indexes
#define GIHT_X 0
#define GIHT_Y 1
#define GIHT_Z 2

#define GIHT_CMD_ROLL 0
#define GIHT_CMD_PITCH 1
#define GIHT_CMD_TZ 2
#define GIHT_CMD_TX 3

// Max Forward and Upward Acceleration
#ifndef GUIDANCE_INDI_MAX_ACC_BODY_X
#define GUIDANCE_INDI_MAX_ACC_BODY_X 2.0f   // Max Forward Acceleration (m/s^2)
#endif

#ifndef GUIDANCE_INDI_MAX_ACC_BODY_Z
#define GUIDANCE_INDI_MAX_ACC_BODY_Z 3.0f   // Max Vertical Acceleration (m/s^2)
#endif

// Wing Aerodynamic Coefficients
#ifndef GUIDANCE_INDI_WING_AREA
#define GUIDANCE_INDI_WING_AREA 0.5f        // Wing Area b*c (m^2)
#endif

#ifndef GUIDANCE_INDI_CL_0
#define GUIDANCE_INDI_CL_0 0.1f             // Zero Alpha Lift Coefficient
#endif

#ifndef GUIDANCE_INDI_CL_ALPHA
#define GUIDANCE_INDI_CL_ALPHA 5.0f         // Lift slope Coefficient (1/rad)
#endif

static constexpr float wing_area = GUIDANCE_INDI_WING_AREA;
static constexpr float CL_0      = GUIDANCE_INDI_CL_0;
static constexpr float CL_alpha  = GUIDANCE_INDI_CL_ALPHA;

static constexpr float air_density = PPRZ_ISA_AIR_DENSITY;

float guidance_indi_max_thr_z = GUIDANCE_INDI_MAX_ACC_BODY_Z * GUIDANCE_INDI_MASS;
float guidance_indi_max_thr_x = GUIDANCE_INDI_MAX_ACC_BODY_X * GUIDANCE_INDI_MASS;


// // Filter Acceleration if used instead of actuator thrust estimates
// #ifndef GUIDANCE_INDI_BODY_FILTER_CUTOFF
// #ifdef GUIDANCE_INDI_FILTER_CUTOFF
// #define GUIDANCE_INDI_BODY_FILTER_CUTOFF GUIDANCE_INDI_FILTER_CUTOFF
// #else
// #define GUIDANCE_INDI_BODY_FILTER_CUTOFF 2.0f
// #endif
// #endif
//
// /**
//  * Call upon entering indi guidance
//  */
// void guidance_indi_tiltrotor_init(void) {
//  float tau_bodyx = 1.f/(2.f*M_PI*GUIDANCE_INDI_BODY_FILTER_CUTOFF);
//  float tau_bodyz = 1.f/(2.f*M_PI*GUIDANCE_INDI_BODY_FILTER_CUTOFF);
//  float sample_time = 1.f / PERIODIC_FREQUENCY;
//  init_butterworth_2_low_pass(&accel_bodyx_filt, tau_bodyx, sample_time, 0.0f);
//  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, 9.81f);
// }
//
// /**
//  * Low pass the accelerometer measurements to remove noise from vibrations.
//  * The roll and pitch also need to be filtered to synchronize them with the
//  * acceleration
//  * Called as a periodic function with PERIODIC_FREQ
//  */
// void guidance_indi_tiltrotor_propagate_filters(void) {
//  // Propagate filters
//  float accelx = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
//  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
//  update_butterworth_2_low_pass(&accel_bodyx_filt, accelx);
//  update_butterworth_2_low_pass(&accel_bodyz_filt, accelz);
// }

/**
 * Calculate aerodynamic lift force along body z axis
 *
 * @param vel    NED velocity vector (m/s) [1x3]
 * @param theta  pitch angle (rad)
 * @return lift force (N)
 */
float guidance_indi_get_lift(struct FloatVect3 vel, float theta)
{
 float V   = sqrtf(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
 float vxy = sqrtf(vel.x*vel.x + vel.y*vel.y);

 if (V <= 4.0f) {
  return 0.0f;
 }

 float gamma = atan2f(-vel.z, vxy);
 float alpha = gamma - theta;
 float dyn   = 0.5f * air_density * V * V;
 float lift  = dyn * wing_area * (CL_0 + CL_alpha * alpha);

 return lift * sinf(alpha);
}

/**
 * Calculate the control effectiveness matrix of partial derivatives of roll, pitch and thrusts
 * w.r.t. the NED Earth Frame using ZXY euler rotation order
 * G = ∂(T_w)/∂[φ, θ, T_z, T_x]
 *
 * @param Gmat Dynamic Matrix [3x5]
 * @param a_diff acceleration errors in earth frame
 * @param v_gih 3D vector to write the control objective v
 */
static void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V])
{
 // Euler Angles
 // ZXY Rotation Order
 float sphi = sinf(roll_filt.o[0]);
 float cphi = cosf(roll_filt.o[0]);
 float stheta = sinf(pitch_filt.o[0]);
 float ctheta = cosf(pitch_filt.o[0]);
 float cpsi  = cosf(yaw_filt);
 float spsi  = sinf(yaw_filt);

 // Lift Estimates
 struct FloatVect3 vel;
 VECT3_COPY(vel, *stateGetSpeedNed_f());
 float lift = guidance_indi_get_lift(vel, pitch_filt.o[0]);

 // Force Estimates
 float Fx = stab_thrust_filt.x;
 float Fz = stab_thrust_filt.z + lift;


 // dφ (Roll)
 Gmat[0][GIHT_CMD_ROLL] = Fx * (-stheta * cphi * spsi) +  Fz * (ctheta * cphi * spsi);
 Gmat[1][GIHT_CMD_ROLL] = Fx * (stheta * cphi * cpsi) +   Fz * (-ctheta * cphi * cpsi);
 Gmat[2][GIHT_CMD_ROLL] = Fx * (stheta * sphi) +          Fz * (-ctheta * spsi);

 // dθ (Pitch)GIHT_CMD_ROLL
 Gmat[0][GIHT_CMD_PITCH] = Fx * (-ctheta * sphi * spsi - stheta * cpsi) + Fz * (-stheta * sphi * spsi + ctheta * cpsi);
 Gmat[1][GIHT_CMD_PITCH] = Fx * (ctheta * sphi * cpsi - stheta * spsi) +  Fz * (stheta * sphi * cpsi + ctheta * spsi);
 Gmat[2][GIHT_CMD_PITCH] = Fx * (-ctheta * cpsi) +                        Fz * (-stheta * cpsi);

 // dTz (Vertical Thrust)
 Gmat[0][GIHT_CMD_TZ] = ctheta * sphi * spsi + stheta * cpsi;
 Gmat[1][GIHT_CMD_TZ] = -ctheta * sphi * cpsi + stheta * spsi;
 Gmat[2][GIHT_CMD_TZ] = ctheta * cphi;

 // dTx (Forward Thrust)
 Gmat[0][GIHT_CMD_TX] = -stheta * sphi * spsi + ctheta * cpsi;
 Gmat[1][GIHT_CMD_TX] = stheta * sphi * cpsi + ctheta * spsi;
 Gmat[2][GIHT_CMD_TX] = -stheta * cphi;


 v_gih[GIHT_X] = a_diff.x;
 v_gih[GIHT_Y] = a_diff.y;
 v_gih[GIHT_Z] = a_diff.z;
}

/**
 * Set WLS Settings (Upper and Lower Bounds, Preference delta u)
 *
 * @param wls WLS solver struct
 */
void guidance_indi_hybrid_set_wls_settings(struct WLS_t *wls)
{
  // Pitch Limits
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  // Pitch Preference
  float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Set lower limits
  wls->u_min[GIHT_CMD_ROLL] = - guidance_indi_max_bank - roll_filt.o[0];       // Min φ (roll)
  wls->u_min[GIHT_CMD_PITCH] = - min_pitch_limit_rad - pitch_filt.o[0];        // Min θ (pitch)
  wls->u_min[GIHT_CMD_TX] = - guidance_indi_max_thr_x;                         // Min Tz (vertical thrust)
  wls->u_min[GIHT_CMD_TZ] = - guidance_indi_max_thr_z;                         // Min Tx (horizontal thrust)

  // Set upper limits
  wls->u_max[GIHT_CMD_ROLL] = guidance_indi_max_bank - roll_filt.o[0];         // Max φ (roll)
  wls->u_max[GIHT_CMD_PITCH] = max_pitch_limit_rad - pitch_filt.o[0];          // Max θ (pitch)
  wls->u_max[GIHT_CMD_TZ] = guidance_indi_max_thr_z;                           // Max Tx (horizontal thrust)
  wls->u_max[GIHT_CMD_TX] = guidance_indi_max_thr_x;                           // Max Tz (vertical thrust)

  // Prefered states
  wls->u_pref[GIHT_CMD_ROLL] = - roll_filt.o[0];                               // Preferred delta φ (roll)
  wls->u_pref[GIHT_CMD_PITCH] = pitch_pref_rad - pitch_filt.o[0];              // Preferred delta θ (pitch)
  wls->u_pref[GIHT_CMD_TZ] = 0.f;                                              // Preferred Tz (horizontal thrust)
  wls->u_pref[GIHT_CMD_TX] = 0.f;                                              // Preferred Tx (vertical thrust)
}
