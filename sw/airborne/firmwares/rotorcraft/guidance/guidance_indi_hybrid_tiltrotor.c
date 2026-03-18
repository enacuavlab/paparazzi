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

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

float gi_pitch_eff_scaling = GUIDANCE_INDI_PITCH_EFF_SCALING;

float bodyx_filter_cutoff = 0.2;
float bodyz_filter_cutoff = 0.2;

Butterworth2LowPass accel_bodyx_filt;
Butterworth2LowPass accel_bodyz_filt;

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

/**
 * Call upon entering indi guidance
 */
void guidance_indi_tiltrotor_init(void) {
  float tau_bodyx = 1.f/(2.f*M_PI*bodyx_filter_cutoff);
  float tau_bodyz = 1.f/(2.f*M_PI*bodyz_filter_cutoff);
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
  /* Pre-calculate sines and cosines from filtered angles */
  const float phi_f = get_butterworth_2_low_pass(&roll_filt);
  const float theta_f = get_butterworth_2_low_pass(&pitch_filt);
  const float psi_f = get_butterworth_2_low_pass(&yaw_filt);
  const float sphi = sinf(phi_f);
  const float cphi = cosf(phi_f);
  const float stheta = sinf(theta_f);
  const float ctheta = cosf(theta_f);
  const float spsi = sinf(psi_f);
  const float cpsi = cosf(psi_f);

  /* Force resultants */
  float fx = GUIDANCE_INDI_MASS * get_butterworth_2_low_pass(&accel_bodyx_filt);
  float fz = GUIDANCE_INDI_MASS * get_butterworth_2_low_pass(&accel_bodyz_filt);

  // get the derivative of the lift wrt to theta
  float dfz = guidance_indi_get_liftd(stateGetAirspeed_f(), theta_f);

  // dPhi (Roll)
  Gmat[GIHT_X][GIHT_CMD_ROLL] = fx * (-spsi * cphi * stheta) + fz * (spsi * cphi * ctheta);
  Gmat[GIHT_Y][GIHT_CMD_ROLL] = fx * (cpsi * cphi * stheta)  + fz * (-cpsi * cphi * ctheta);
  Gmat[GIHT_Z][GIHT_CMD_ROLL] = fx * (sphi * stheta)         + fz * (-sphi * ctheta);

  // dTheta (Pitch)
  Gmat[GIHT_X][GIHT_CMD_PITCH] =
    fx * (-cpsi * stheta - spsi * sphi * ctheta) +
    fz * (cpsi * ctheta - spsi * sphi * stheta) * gi_pitch_eff_scaling +
    dfz * (cpsi * stheta + spsi * sphi * ctheta);
  Gmat[GIHT_Y][GIHT_CMD_PITCH] =
    fx * (-spsi * stheta + cpsi * sphi * ctheta) +
    fz * (spsi * ctheta + cpsi * sphi * stheta) * gi_pitch_eff_scaling +
    dfz * (spsi * stheta - cpsi * sphi * ctheta);
  Gmat[GIHT_Z][GIHT_CMD_PITCH] =
    fx * (-cphi * ctheta) +
    fz * (-cphi * stheta) * gi_pitch_eff_scaling +
    dfz * cphi * ctheta;

  // dfz
  Gmat[GIHT_X][GIHT_CMD_FZ] = (cpsi * stheta + spsi * sphi * ctheta);
  Gmat[GIHT_Y][GIHT_CMD_FZ] = (spsi * stheta - cpsi * sphi * ctheta);
  Gmat[GIHT_Z][GIHT_CMD_FZ] = cphi * ctheta;

  // dfx
  Gmat[GIHT_X][GIHT_CMD_FX]  =  (cpsi * ctheta - spsi * sphi * stheta);
  Gmat[GIHT_Y][GIHT_CMD_FX]  =  (spsi * ctheta + cpsi * sphi * stheta);
  Gmat[GIHT_Z][GIHT_CMD_FX]  = -stheta * cphi;

  body_v[GIHT_X] =  a_diff.x;
  body_v[GIHT_Y] =  a_diff.y;
  body_v[GIHT_Z] =  a_diff.z;
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
  const float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  const float min_pitch_limit_rad = RadOfDeg(guidance_indi_min_pitch);

  // Weights evolution
  const float Wu_original[GUIDANCE_INDI_HYBRID_U] = GUIDANCE_INDI_WLS_WU;
  const float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;

  // FIXME improve this part
  wls_guid_p.Wu[GIHT_CMD_PITCH] = Wu_original[GIHT_CMD_PITCH] + 5 * T1.as;
  wls_guid_p.Wu[GIHT_CMD_FX] = Wu_original[GIHT_CMD_FX] + 300 * (1 - expf(T1.as / 4.0));
  wls_guid_p.Wv[0] = Wv_original[0] + 3 * T1.as; //maybe could evolute with tilt angle

  // Set lower limits
  wls_guid_p.u_min[GIHT_CMD_ROLL]  = -guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_min[GIHT_CMD_PITCH] = min_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_min[GIHT_CMD_FZ]    = -20 - GUIDANCE_INDI_MASS * accel_bodyz_filt.o[0];
  wls_guid_p.u_min[GIHT_CMD_FX]    = 0 - GUIDANCE_INDI_MASS * accel_bodyx_filt.o[0];

  // Set upper limits
  wls_guid_p.u_max[GIHT_CMD_ROLL]  = guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_max[GIHT_CMD_PITCH] = max_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_max[GIHT_CMD_FZ]    = 0 - GUIDANCE_INDI_MASS * accel_bodyz_filt.o[0];
  wls_guid_p.u_max[GIHT_CMD_FX]    = 20 - GUIDANCE_INDI_MASS * accel_bodyx_filt.o[0];

  // Set prefered states
  wls_guid_p.u_pref[GIHT_CMD_ROLL]  = -roll_angle; // prefered delta roll angle
  wls_guid_p.u_pref[GIHT_CMD_PITCH] = -pitch_angle; // prefered delta pitch angle
  wls_guid_p.u_pref[GIHT_CMD_FZ]    =  GUIDANCE_INDI_MASS * accel_bodyz_filt.o[0];
  wls_guid_p.u_pref[GIHT_CMD_FX]    =  GUIDANCE_INDI_MASS * accel_bodyx_filt.o[0];
}

