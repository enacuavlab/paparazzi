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
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_quadplane.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "autopilot.h"
#include "generated/modules.h"
#include "modules/ctrl/eff_scheduling_quadplane.h"

#ifndef COMMAND_THRUST_X
#error "Quadplanes require a forward thrust actuator"
#endif

#ifndef GUIDANCE_INDI_PUSHER_INDEX
#error "You need to define GUIDANCE_INDI_PUSHER_INDEX"
#endif

#ifndef VEHICLE_ROTOR_K_T_PPRZ
#error "VEHICLE_K_T_PPRZ [2x1 float] not defined in airframe file — {b [N/pprz], c [N/pprz^2]}"
#endif
#ifndef VEHICLE_ROTOR_ROT_X
#error "VEHICLE_ROTOR_ROT_X not defined in airframe file"
#endif
#ifndef VEHICLE_ROTOR_ROT_Y
#error "VEHICLE_ROTOR_ROT_Y not defined in airframe file"
#endif
#ifndef VEHICLE_ROTOR_ROT_Z
#error "VEHICLE_ROTOR_ROT_Z not defined in airframe file"
#endif

#ifndef GUIDANCE_INDI_BODYZ_FILTER_CUTOFF
#ifdef GUIDANCE_INDI_FILTER_CUTOFF
#define GUIDANCE_INDI_BODYZ_FILTER_CUTOFF GUIDANCE_INDI_FILTER_CUTOFF
#else
#define GUIDANCE_INDI_BODYZ_FILTER_CUTOFF 3.0
#endif
#endif

// preferred pitch when wing-borne [deg]
#ifndef GUIDANCE_INDI_PITCH_PREF_CRUISE_DEG
#define GUIDANCE_INDI_PITCH_PREF_CRUISE_DEG 0.f   
#endif
// extra WLS weight on hover thrust when wing-borne
#ifndef GUIDANCE_INDI_THRUST_WU_GAIN
#define GUIDANCE_INDI_THRUST_WU_GAIN 10.f         
#endif
// extra WLS weight on pitch in hover (pusher does the x translation)
#ifndef GUIDANCE_INDI_PITCH_HOVER_WU
#define GUIDANCE_INDI_PITCH_HOVER_WU 10.f   
#endif
/* Same thresholds and names as the default rotorcraft detection (autopilot_firmware.c),
 * so that an airframe setting applies to both */
#ifndef AUTOPILOT_IN_FLIGHT_TIME
#define AUTOPILOT_IN_FLIGHT_TIME 20
#endif
#ifndef AUTOPILOT_IN_FLIGHT_MIN_SPEED
#define AUTOPILOT_IN_FLIGHT_MIN_SPEED 0.2
#endif
#ifndef AUTOPILOT_IN_FLIGHT_MIN_ACCEL
#define AUTOPILOT_IN_FLIGHT_MIN_ACCEL 2.0
#endif
#ifndef AUTOPILOT_IN_FLIGHT_MIN_THRUST
#define AUTOPILOT_IN_FLIGHT_MIN_THRUST 500
#endif
/* Quadplane guard: a wing-borne flight (hover rotors off) always has some ground speed */
#ifndef CMQ_LANDED_MAX_GROUNDSPEED
#define CMQ_LANDED_MAX_GROUNDSPEED 1.f
#endif

Butterworth2LowPass accel_bodyz_filt;

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_quadplane_init(void) {
  float tau_bodyz = 1.0/(2.0*M_PI*GUIDANCE_INDI_BODYZ_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, -9.81);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_quadplane_propagate_filters(void) {
   // Propagate filter for thrust/lift estimate
  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
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

  struct FloatEulers eulers_filtered;
  float_eulers_of_quat_zxy(&eulers_filtered, &quat_filt.quat);
  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_filtered.phi);
  float cphi = cosf(eulers_filtered.phi);
  float stheta = sinf(eulers_filtered.theta);
  float ctheta = cosf(eulers_filtered.theta);
  float spsi = sinf(eulers_filtered.psi);
  float cpsi = cosf(eulers_filtered.psi);

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float lift_thrust_bz = accel_bodyz_filt.o[0]; // Sum of lift and thrust in boxy z axis (level flight)

  // get the derivative of the lift wrt to theta
  float airspeed_for_lift = quadplane_eff_sched_v.airspeed;
  float liftd = guidance_indi_get_liftd(airspeed_for_lift, eulers_filtered.theta);

  // ZXY Euler order
  Gmat[0][0] =  0.0f;
  Gmat[1][0] = -cphi*ctheta*lift_thrust_bz;
  Gmat[2][0] = -ctheta*sphi*lift_thrust_bz;

  Gmat[0][1] =  ctheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING;
  Gmat[1][1] =  sphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*liftd;
  Gmat[2][1] = -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;

  Gmat[0][2] =  stheta;
  Gmat[1][2] = -sphi*ctheta;
  Gmat[2][2] =  cphi*ctheta;

  Gmat[0][3] =  ctheta;
  Gmat[1][3] =  sphi*stheta;
  Gmat[2][3] = -cphi*stheta;
  /* Pusher lift only counted when wing-borne: removes the hover null-space drift */
  Gmat[2][3] *= (1.f - quadplane_eff_rotor_authority);
  // Make this term zero to prevent switching 'exploits'
  // Gmat[2][3] = 0;

  // Convert acceleration error to body axis system
  body_v[0] =  cpsi * a_diff.x + spsi * a_diff.y;
  body_v[1] = -spsi * a_diff.x + cpsi * a_diff.y;
  body_v[2] =  a_diff.z;
}


void WEAK guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  /* Hover rotor authority from eff_scheduling: 1 = quad, 0 = wing-borne */
  const float a = quadplane_eff_rotor_authority;

  /* ---------- Pitch preference, scheduled from hover to cruise ---------- */
  const float pitch_pref_rad = a * RadOfDeg(guidance_indi_pitch_pref_deg)
                             + (1.f - a) * RadOfDeg(GUIDANCE_INDI_PITCH_PREF_CRUISE_DEG);

  /* ---------- Collective thrust effectiveness (unchanged physics) ---------- */
  const float rotor_rot_x[4] = VEHICLE_ROTOR_ROT_X;
  const float rotor_rot_y[4] = VEHICLE_ROTOR_ROT_Y;
  const float rotor_k_t_pprz[2] = VEHICLE_ROTOR_K_T_PPRZ;
  float u_col = (float)stabilization.cmd[COMMAND_THRUST];   /* mean of Z thrusters */
  Bound(u_col, 0.f, (float)MAX_PPRZ);
  const float rotor_dT_du = rotor_k_t_pprz[0] + 2.f * rotor_k_t_pprz[1] * u_col;
  float thrust_z_eff = 0.f;                                  /* [m/s^2 per pprz], negative */
  for (uint8_t i = 0; i < 4; i++) {
    thrust_z_eff += -cosf(rotor_rot_y[i]) * cosf(rotor_rot_x[i]) * rotor_dT_du;
  }
  thrust_z_eff /= VEHICLE_MASS;

  /* Same rotor range as the inner WLS (stabilization_indi_set_wls_settings) */
  const float u_top = (a > 0.f) ? VEHICLE_MOTOR_IDLE + a * (MAX_PPRZ - VEHICLE_MOTOR_IDLE) : 0.f;
  const float u_bot = (a > 0.f) ? VEHICLE_MOTOR_IDLE : 0.f;

  /* ---------- Roll ---------- */
  wls_guid_p.u_min[0]  = -guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_max[0]  =  guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_pref[0] = -roll_angle;

  /* ---------- Pitch: kept level in hover, free when wing-borne ---------- */
  wls_guid_p.u_min[1]  = RadOfDeg(GUIDANCE_INDI_MIN_PITCH) - pitch_angle;
  wls_guid_p.u_max[1]  = RadOfDeg(GUIDANCE_INDI_MAX_PITCH) - pitch_angle;
  wls_guid_p.u_pref[1] = pitch_pref_rad - pitch_angle;
  wls_guid_p.Wu[1]     = 1.f + a * GUIDANCE_INDI_PITCH_HOVER_WU;

  /* ---------- Hover thrust (body z): bounded by rotor authority ---------- */
  wls_guid_p.u_min[2]  = (u_top - u_col) * thrust_z_eff;   /* more upward thrust */
  wls_guid_p.u_max[2]  = (u_bot - u_col) * thrust_z_eff;   /* less upward thrust */
  wls_guid_p.u_pref[2] = (1.f - a) * wls_guid_p.u_max[2]; //wls_guid_p.u_max[2];              /* low thrust preferred */
  wls_guid_p.Wu[2]     = 1.f + (1.f - a) * GUIDANCE_INDI_THRUST_WU_GAIN;

  /* ---------- Pusher: main x actuator in every flight phase ---------- */
  const float pusher_eff = g1g2[4][GUIDANCE_INDI_PUSHER_INDEX];
  float u_x = actuator_state_filt_vect[QUADPLANE_ACT_PUSHER];
  Bound(u_x, 0.f, (float)MAX_PPRZ);
  if (quadplane_eff_disable_pusher || quadplane_eff_pusher_forced) {
    /* Ground, or pusher owned by the mixer (manual FWD / legacy transition) */
    wls_guid_p.u_min[3]  = 0.f;
    wls_guid_p.u_max[3]  = 0.f;
    wls_guid_p.u_pref[3] = 0.f;
  } else {
    wls_guid_p.u_min[3]  = -u_x * pusher_eff;
    wls_guid_p.u_max[3]  = ((float)MAX_PPRZ - u_x) * pusher_eff;
    /* Pusher takes the whole x acceleration demand, within its range */
    float pref_x = body_v[0];
    Bound(pref_x, wls_guid_p.u_min[3], wls_guid_p.u_max[3]);
    wls_guid_p.u_pref[3] = pref_x;
  }
  wls_guid_p.Wu[3] = 1.f;
}



/**
 * Landing detection, same rule as the default rotorcraft one (low hover thrust,
 * no vertical speed, no vertical acceleration during AUTOPILOT_IN_FLIGHT_TIME calls),
 * plus a ground speed guard: in cruise the hover thrust is zero but the aircraft moves.
 */
bool autopilot_in_flight_end_detection(bool motors_on)
{
  static uint32_t landed_counter = 0;

  if (!motors_on) {
    landed_counter = 0;
    return true;
  }

  if ((stabilization.cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
      (fabsf(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
      (fabsf(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL) &&
      (stateGetHorizontalSpeedNorm_f() < CMQ_LANDED_MAX_GROUNDSPEED)) {
    landed_counter++;
    if (landed_counter >= AUTOPILOT_IN_FLIGHT_TIME) {
      landed_counter = 0;
      return true;
    }
  } else {
    landed_counter = 0;
  }
  return false;
}
