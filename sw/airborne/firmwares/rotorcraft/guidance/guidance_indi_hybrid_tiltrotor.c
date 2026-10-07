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
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "math/pprz_isa.h"
#include "modules/ctrl/eff_scheduling_atlas.h"
#ifdef RADIO_TILT
#include "modules/radio_control/radio_control.h"
#endif


static const float air_density = PPRZ_ISA_AIR_DENSITY;

/* Aerodynamic model */
static const float wing_area   = 0.072f;   /* [m^2] */
static const float CL_0        = 0.1f;
static const float CL_alpha    = 2.0f;     /* [1/rad] */
static const float CL_min      = -0.5f;
static const float CL_max      = 1.2f;
static const float S_CD        = 0.06f;    /* drag area S*CD [m^2] */
static const float lift_v_on   = 4.0f;     /* wing term fades in over this band [m/s] */
static const float lift_v_full = 6.0f;

/* Fz handed to the outer WLS is held in this band: it must stay clearly negative
 * or the roll/pitch columns of G collapse and then invert. */
static const float Fz_min = -20.0f;        /* [m/s^2] */
static const float Fz_max = -5.0f;

static const float pitch_pref_max_incr UNUSED = 0.0873f;  /* 5 deg */

float guidance_indi_max_thr_z = GUIDANCE_INDI_MAX_ACC_BODY_Z * GUIDANCE_INDI_MASS;
float guidance_indi_max_thr_x = GUIDANCE_INDI_MAX_ACC_BODY_X * GUIDANCE_INDI_MASS;

/* Outer-WLS actuator weights, settable so the pitch/tilt split can be modified
 */
float guidance_indi_wu_roll  = GUIDANCE_INDI_WU_ROLL;
float guidance_indi_wu_pitch = GUIDANCE_INDI_WU_PITCH;
float guidance_indi_wu_tz    = GUIDANCE_INDI_WU_TZ;
float guidance_indi_wu_tx    = GUIDANCE_INDI_WU_TX;


// // Filter Acceleration if used instead of thrust estimates
// #ifndef GUIDANCE_INDI_BODY_FILTER_CUTOFF
// #define GUIDANCE_INDI_BODY_FILTER_CUTOFF 2.0f
// #endif
// Butterworth2LowPass accel_bodyx_filt;
// Butterworth2LowPass accel_bodyz_filt;
//
// void guidance_indi_tiltrotor_init(void) {
//   float tau = 1.f / (2.f * M_PI * GUIDANCE_INDI_BODY_FILTER_CUTOFF);
//   float dt  = 1.f / PERIODIC_FREQUENCY;
//   init_butterworth_2_low_pass(&accel_bodyx_filt, tau, dt,  0.0f);
//   init_butterworth_2_low_pass(&accel_bodyz_filt, tau, dt, -9.81f);  // body z is down
// }
//
// void guidance_indi_tiltrotor_propagate_filters(void) {
//   update_butterworth_2_low_pass(&accel_bodyx_filt, ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x));
//   update_butterworth_2_low_pass(&accel_bodyz_filt, ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z));
// }

// Wing term fade-in over the forward (body x) speed
static float wing_fade(float u)
{
 float fade = (u - lift_v_on) / (lift_v_full - lift_v_on);
 Bound(fade, 0.0f, 1.0f);
 return fade * fade * (3.0f - 2.0f * fade);
}

/**
 * Derivative of the wing lift w.r.t. pitch, as specific force (m/s^2 per rad).
 * Negative as in the other hybrids: pitching up adds lift, which acts along -z.
 *
 * @param airspeed  forward (body x) speed (m/s)
 */
float guidance_indi_get_liftd(float airspeed, float theta UNUSED)
{
 const float fade = wing_fade(airspeed);
 if (fade <= 0.0f) {
  return 0.0f;
 }
 float liftd = -fade * 0.5f * air_density * airspeed * airspeed * wing_area * CL_alpha / GUIDANCE_INDI_MASS;
 Bound(liftd, -150.0f, 0.0f);
 return liftd;
}

/**
 * Aerodynamic specific force, resolved from the wind frame onto the body axes
 *
 * @param vel    NED velocity vector (m/s) [1x3]
 * @param eul    body attitude, ZXY euler order (rad)
 * @param fx     body-x specific force (m/s^2)
 * @param fz     body-z specific force (m/s^2)
 * @param liftd  lift derivative w.r.t. pitch (m/s^2 per rad), zero while CL is clamped
 * @param aoa    angle of attack (rad)
 */
void guidance_indi_get_aero(struct FloatVect3 vel, struct FloatEulers *eul,
                            float *fx, float *fz, float *liftd, float *aoa)
{
 *fx = 0.0f;
 *fz = 0.0f;
 *liftd = 0.0f;
 *aoa = 0.0f;

 const float sphi = sinf(eul->phi),   cphi = cosf(eul->phi);
 const float sth  = sinf(eul->theta), cth  = cosf(eul->theta);
 const float spsi = sinf(eul->psi),   cpsi = cosf(eul->psi);

 // Relative wind resolved on the body x and z axes (ZXY, same columns as Tx / Tz)
 const float u = (-sth*sphi*spsi + cth*cpsi) * vel.x
               + ( sth*sphi*cpsi + cth*spsi) * vel.y
               + (-sth*cphi)                 * vel.z;
 const float w = ( cth*sphi*spsi + sth*cpsi) * vel.x
               + (-cth*sphi*cpsi + sth*spsi) * vel.y
               + ( cth*cphi)                 * vel.z;

 // Lift and drag run on the forward (body x) speed only
 if (!(u > 0.1f)) {
  return;
 }

 const float alpha = atan2f(w, u);
 const float dyn   = 0.5f * air_density * u * u;

 float drag = dyn * S_CD;
 float lift = 0.0f;

 const float fade = wing_fade(u);
 if (fade > 0.0f) {
  float CL = CL_0 + CL_alpha * alpha;
  // Slope tapers to zero over the last 0.2 of CL before a clamp
  float slope = Min(CL - CL_min, CL_max - CL) / 0.2f;
  Bound(slope, 0.0f, 1.0f);
  *liftd = slope * guidance_indi_get_liftd(u, eul->theta);
  Bound(CL, CL_min, CL_max);
  lift = fade * dyn * wing_area * CL;
 }
 *aoa = alpha;

 const float ca = cosf(alpha), sa = sinf(alpha);
 *fx = (-drag * ca + lift * sa) / GUIDANCE_INDI_MASS;
 *fz = (-drag * sa - lift * ca) / GUIDANCE_INDI_MASS;
}

/**
 * Calculate the control effectiveness matrix of partial derivatives of roll, pitch and thrusts
 * w.r.t. the NED Earth Frame using ZXY euler rotation order
 * G = ∂(T_w)/∂[φ, θ, T_z, T_x]
 *
 * @param Gmat Dynamic Matrix [3x4]
 * @param a_diff acceleration errors in earth frame
 * @param v_gih 3D vector to write the control objective v
 */
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V])
{
 struct FloatEulers eulers_filtered;
 float_eulers_of_quat_zxy(&eulers_filtered, &quat_filt.quat);
 // Euler Angles
 // ZXY Rotation Order
 float sphi = sinf(eulers_filtered.phi);
 float cphi = cosf(eulers_filtered.phi);
 float stheta = sinf(eulers_filtered.theta);
 float ctheta = cosf(eulers_filtered.theta);
 float cpsi  = cosf(eulers_filtered.psi);
 float spsi  = sinf(eulers_filtered.psi);

 // Aerodynamic force estimate, wind frame rotated onto the body axes
 struct FloatVect3 vel;
 VECT3_COPY(vel, *stateGetSpeedNed_f());
 float aero_x, aero_z, liftd, aoa;
 guidance_indi_get_aero(vel, &eulers_filtered, &aero_x, &aero_z, &liftd, &aoa);

 // Thrust Estimates
 const float T_r = atlas_eff_sched_v.T[0] + atlas_eff_sched_v.T[1];
 const float T_l = atlas_eff_sched_v.T[2] + atlas_eff_sched_v.T[3];
 const float Tx =  (T_r * atlas_eff_sched_v.sin_ar + T_l * atlas_eff_sched_v.sin_al) / atlas_eff_sched_p.m;
 const float Tz = -(T_r * atlas_eff_sched_v.cos_ar + T_l * atlas_eff_sched_v.cos_al) / atlas_eff_sched_p.m;

 // Roll turns the thrust and the aerodynamic force together
 float Fx = Tx + aero_x;
 float Fz = Tz + aero_z;
 Bound(Fz, Fz_min, Fz_max);

 // Pitch turns the thrust only: lift stays normal to the wind and grows with angle of attack
 float Px = Tx - liftd * cosf(aoa);
 float Pz = Tz - liftd * sinf(aoa);
 Bound(Pz, Fz_min, Fz_max);

 // // Measured Force
 // float Fx = accel_bodyx_filt.o[0];
 // float Fz = accel_bodyz_filt.o[0];
 // if (Fz > -5.f) { Fz = -5.f; }


 // dφ (Roll)
 Gmat[0][GIHT_CMD_ROLL] = Fx * (-stheta * cphi * spsi) +  Fz * (ctheta * cphi * spsi);
 Gmat[1][GIHT_CMD_ROLL] = Fx * (stheta * cphi * cpsi) +   Fz * (-ctheta * cphi * cpsi);
 Gmat[2][GIHT_CMD_ROLL] = Fx * (stheta * sphi) +          Fz * (-ctheta * sphi);

 // dθ (Pitch)
 Gmat[0][GIHT_CMD_PITCH] = Px * (-ctheta * sphi * spsi - stheta * cpsi) + Pz * (-stheta * sphi * spsi + ctheta * cpsi);
 Gmat[1][GIHT_CMD_PITCH] = Px * (ctheta * sphi * cpsi - stheta * spsi) +  Pz * (stheta * sphi * cpsi + ctheta * spsi);
 Gmat[2][GIHT_CMD_PITCH] = Px * (-ctheta * cphi) +                        Pz * (-stheta * cphi);

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
 * Set outer-loop WLS bounds and preferences.
 *
 * @param body_v       speed setpoint in the heading frame [3] (unused here)
 * @param roll_angle   current roll  [rad]
 * @param pitch_angle  current pitch [rad]
 */
void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
  float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;
  wls_guid_p.Wv[GIHT_X] = Wv_original[GIHT_X];
  wls_guid_p.Wv[GIHT_Y] = Wv_original[GIHT_Y];
  wls_guid_p.Wv[GIHT_Z] = Wv_original[GIHT_Z];

  wls_guid_p.Wu[GIHT_CMD_ROLL]  = guidance_indi_wu_roll;
  wls_guid_p.Wu[GIHT_CMD_PITCH] = guidance_indi_wu_pitch;
  wls_guid_p.Wu[GIHT_CMD_TZ]    = guidance_indi_wu_tz;
  wls_guid_p.Wu[GIHT_CMD_TX]    = guidance_indi_wu_tx;

  const float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  const float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  guidance_indi_pitch_pref_deg = GUIDANCE_INDI_PITCH_PREF_DEG;
#ifdef RADIO_TILT
  // In GUIDED the tilt wheel sets the pitch preference: wheel min = RC_MIN
  // (-10 deg), center = 0 deg, wheel max = RC_MAX (+20 deg). Falls back to
  // the compile-time preference in other modes or when RC is lost.
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED && radio_control.status == RC_OK) {
    float wheel = (float)radio_control_get(RADIO_TILT) / (float)MAX_PPRZ;
    BoundAbs(wheel, 1.0f);
    guidance_indi_pitch_pref_deg = (wheel >= 0.0f)
        ? wheel * GUIDANCE_INDI_PITCH_PREF_RC_MAX_DEG
        : -wheel * GUIDANCE_INDI_PITCH_PREF_RC_MIN_DEG;
  }
#endif
  const float pitch_pref_rad UNUSED = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Roll limits
  wls_guid_p.u_min[GIHT_CMD_ROLL]  = -guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_max[GIHT_CMD_ROLL]  =  guidance_indi_max_bank - roll_angle;

  // Pitch limits
  wls_guid_p.u_min[GIHT_CMD_PITCH] = min_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_max[GIHT_CMD_PITCH] = max_pitch_limit_rad - pitch_angle;

  // Dynamic vertical thrust headroom
  float du_min_thrust_z = 0.f, du_max_thrust_z = 0.f;
  for (int i = 0; i < 4; i++) {
    float ca = (i < 2) ? atlas_eff_sched_v.cos_ar : atlas_eff_sched_v.cos_al;
    du_max_thrust_z += (MAX_PPRZ - actuator_state_filt_vect[i]) * atlas_eff_sched_v.dT_dpprz[i] * ca / atlas_eff_sched_p.m;
    du_min_thrust_z +=           (-actuator_state_filt_vect[i]) * atlas_eff_sched_v.dT_dpprz[i] * ca / atlas_eff_sched_p.m;
  }
  Bound(du_min_thrust_z,   6.f, 0.f);
  Bound(du_max_thrust_z,   0.f, -10.f);

  wls_guid_p.u_min[GIHT_CMD_TZ] = du_max_thrust_z;  // Min Tz: most climb   (<=0)
  wls_guid_p.u_max[GIHT_CMD_TZ] = du_min_thrust_z;  // Max Tz: most descend (>=0)

  // Dynamic forward thrust headroom
  // (a) motor channel: more/less thrust along the current tilt direction
  float du_min_thrust_x = 0.f, du_max_thrust_x = 0.f;
  for (int i = 0; i < 4; i++) {
    float sa = (i < 2) ? atlas_eff_sched_v.sin_ar : atlas_eff_sched_v.sin_al;
    du_max_thrust_x += (MAX_PPRZ - actuator_state_filt_vect[i]) * atlas_eff_sched_v.dT_dpprz[i] * sa / atlas_eff_sched_p.m;
    du_min_thrust_x +=           (-actuator_state_filt_vect[i]) * atlas_eff_sched_v.dT_dpprz[i] * sa / atlas_eff_sched_p.m;
  }
  // (b) tilt channel: rotating the current thrust vector with the tilt servos.
  // Without this the bounds collapse to [0,0] at alpha = 0 and the guidance
  // can never command Tx, so the tilt never engages (hover deadlock).
  if (!atlas_eff_disable_tilt) {
    const float s_max = sinf(atlas_eff_sched_p.alpha_max);
    const float s_min = sinf(atlas_eff_sched_p.alpha_min);
    for (int i = 0; i < 4; i++) {
      float sa = (i < 2) ? atlas_eff_sched_v.sin_ar : atlas_eff_sched_v.sin_al;
      du_max_thrust_x += atlas_eff_sched_v.T[i] * (s_max - sa) / atlas_eff_sched_p.m;
      du_min_thrust_x += atlas_eff_sched_v.T[i] * (s_min - sa) / atlas_eff_sched_p.m;
    }
  }
  Bound(du_min_thrust_x, -1.f, 0.f);
  Bound(du_max_thrust_x,   0.f, 4.f);

  wls_guid_p.u_min[GIHT_CMD_TX] = du_min_thrust_x;  // Min Tx (forward thrust)
  wls_guid_p.u_max[GIHT_CMD_TX] = du_max_thrust_x;  // Max Tx (forward thrust)

  wls_guid_p.u_pref[GIHT_CMD_ROLL]  = 0.f;
  wls_guid_p.u_pref[GIHT_CMD_TZ]    = 0.f;
  wls_guid_p.u_pref[GIHT_CMD_TX]    = 0.f;

  /* No pitch preference */
  // wls_guid_p.u_pref[GIHT_CMD_PITCH] = 0.f;

  /* Pitch preference */
   wls_guid_p.u_pref[GIHT_CMD_PITCH] = pitch_pref_rad - pitch_angle;
   BoundAbs(wls_guid_p.u_pref[GIHT_CMD_PITCH], pitch_pref_max_incr);
}
