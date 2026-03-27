/*
 * Copyright (C) 2025 Ramon Revilla Bouso <ramonrevilla21@gmail.com>
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

// Input and output indexes
#define GIHT_X 0
#define GIHT_Y 1
#define GIHT_Z 2
#define GIHT_CMD_ROLL   0
#define GIHT_CMD_PITCH  1
#define GIHT_CMD_FZ     2
#define GIHT_CMD_FX     3

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0f
#endif

#ifndef GUIDANCE_INDI_BODY_FILTER_CUTOFF
#ifdef GUIDANCE_INDI_FILTER_CUTOFF
#define GUIDANCE_INDI_BODY_FILTER_CUTOFF GUIDANCE_INDI_FILTER_CUTOFF
#else
#define GUIDANCE_INDI_BODY_FILTER_CUTOFF 2.0f
#endif
#endif

float gi_pitch_eff_scaling = GUIDANCE_INDI_PITCH_EFF_SCALING;

static Butterworth2LowPass accel_bodyx_filt;
static Butterworth2LowPass accel_bodyz_filt;

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

/**
 * Call upon entering indi guidance
 */
void guidance_indi_tiltrotor_init(void) {
  float tau_bodyx = 1.f/(2.f*M_PI*GUIDANCE_INDI_BODY_FILTER_CUTOFF);
  float tau_bodyz = 1.f/(2.f*M_PI*GUIDANCE_INDI_BODY_FILTER_CUTOFF);
  float sample_time = 1.f / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_bodyx_filt, tau_bodyx, sample_time, 0.0f);
  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, 9.81f);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_tiltrotor_propagate_filters(void) {
   // Propagate filters
  float accelx = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  update_butterworth_2_low_pass(&accel_bodyx_filt, accelx);
  update_butterworth_2_low_pass(&accel_bodyz_filt, accelz);
}

/**
 * Perform WLS
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float body_v[GUIDANCE_INDI_HYBRID_V]) {
  /* Pre-calculate sines and cosines from filtered angles (in zxy order) */
  const float phi_f = get_butterworth_2_low_pass(&roll_filt);
  const float theta_f = get_butterworth_2_low_pass(&pitch_filt);
  //const float psi_f = get_butterworth_2_low_pass(&yaw_filt); FIXME not a valid filtering !!!
  const float psi_f = stateGetNedToBodyEulers_f()->psi; // FIXME temporary fix, use unfiltered psi (same for ZYX and ZXY eulers)
  const float sphi = sinf(phi_f);
  const float cphi = cosf(phi_f);
  const float stheta = sinf(theta_f);
  const float ctheta = cosf(theta_f);
  const float spsi = sinf(psi_f);
  const float cpsi = cosf(psi_f);

  /* Force resultants */
  // FIXME forces are estimated from accelerometers, e.g. specific forces. Should we multiply by the mass or not ?
  float fx = GUIDANCE_INDI_MASS * get_butterworth_2_low_pass(&accel_bodyx_filt); // FIXME assume 0 ?
  float fz = GUIDANCE_INDI_MASS * get_butterworth_2_low_pass(&accel_bodyz_filt);

  // get the derivative of the lift wrt to theta
  float dfz = guidance_indi_get_liftd(stateGetAirspeed_f(), theta_f);

  // dPhi (Roll)
  Gmat[GIHT_X][GIHT_CMD_ROLL] = 0.f;
  Gmat[GIHT_Y][GIHT_CMD_ROLL] = fx * (cphi * stheta) + fz * (-cphi * ctheta);
  Gmat[GIHT_Z][GIHT_CMD_ROLL] = fx * (sphi * stheta) + fz * (-sphi * ctheta);

  // dTheta (Pitch)
  Gmat[GIHT_X][GIHT_CMD_PITCH] = fx * (-stheta) + fz * ctheta * gi_pitch_eff_scaling + dfz * stheta;
  Gmat[GIHT_Y][GIHT_CMD_PITCH] = fx * (sphi * ctheta) + fz * (sphi * stheta) * gi_pitch_eff_scaling + dfz * (-sphi * ctheta);
  Gmat[GIHT_Z][GIHT_CMD_PITCH] = fx * (-cphi * ctheta) + fz * (-cphi * stheta) * gi_pitch_eff_scaling + dfz * cphi * ctheta;

  // dfz
  Gmat[GIHT_X][GIHT_CMD_FZ] = stheta;
  Gmat[GIHT_Y][GIHT_CMD_FZ] = -sphi * ctheta;
  Gmat[GIHT_Z][GIHT_CMD_FZ] = cphi * ctheta;

  // dfx
  Gmat[GIHT_X][GIHT_CMD_FX] =  ctheta;
  Gmat[GIHT_Y][GIHT_CMD_FX] =  sphi * stheta;
  Gmat[GIHT_Z][GIHT_CMD_FX] = -cphi * stheta;

  body_v[GIHT_X] =  cpsi * a_diff.x + spsi * a_diff.y;
  body_v[GIHT_Y] = -spsi * a_diff.x + cpsi * a_diff.y;
  body_v[GIHT_Z] =  a_diff.z;
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
  const float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  const float min_pitch_limit_rad = RadOfDeg(guidance_indi_min_pitch);
  const float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Weights evolution
  // FIXME improve this part
  //const float Wu_original[GUIDANCE_INDI_HYBRID_U] = GUIDANCE_INDI_WLS_WU;
  //const float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;
  //wls_guid_p.Wu[GIHT_CMD_PITCH] = Wu_original[GIHT_CMD_PITCH] + 5 * T1.as;
  //wls_guid_p.Wu[GIHT_CMD_FX] = Wu_original[GIHT_CMD_FX] + 300. * (1.0 - expf(T1.as / 4.0));
  //wls_guid_p.Wv[0] = Wv_original[0] + 3 * T1.as; //maybe could evolute with tilt angle

  // Estimated thrust at max throttle on Z axis
  float max_thrust_z = stabilization_indi_get_thrust(MAX_PPRZ, 2);
  float max_thrust_x = stabilization_indi_get_thrust(MAX_PPRZ, 0);

  // Set lower limits
  wls_guid_p.u_min[GIHT_CMD_ROLL]  = -guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_min[GIHT_CMD_PITCH] = min_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_min[GIHT_CMD_FZ]    = max_thrust_z - stab_thrust_filt.z;
  wls_guid_p.u_min[GIHT_CMD_FX]    = max_thrust_x - stab_thrust_filt.x;

  // Set upper limits
  wls_guid_p.u_max[GIHT_CMD_ROLL]  = guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_max[GIHT_CMD_PITCH] = max_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_max[GIHT_CMD_FZ]    = -stab_thrust_filt.z;
  wls_guid_p.u_max[GIHT_CMD_FX]    = -stab_thrust_filt.x;

  // Set prefered states
  wls_guid_p.u_pref[GIHT_CMD_ROLL]  = -roll_angle;                    // prefered delta roll angle
  wls_guid_p.u_pref[GIHT_CMD_PITCH] = -pitch_angle + pitch_pref_rad;  // prefered delta pitch angle
  wls_guid_p.u_pref[GIHT_CMD_FZ]    =  wls_guid_p.u_max[2];           // low thrust better for efficiency TODO check this
  wls_guid_p.u_pref[GIHT_CMD_FX]    =  body_v[0];                     // solve body acceleration
}

