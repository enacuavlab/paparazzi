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
 * @file modules/ctrl/circ_trajectory.c
 *
 * Circular reference trajectory generator for the INDI hybrid guidance
 *
 * The circle is parametrised by an angle theta(t). The angular rate that
 * gives the maximum tangential speed is given by: omega = v_max / r
 *
 *
 * Modes:
 *   HOVER (default)  hold here: the position and heading captured on entry
 *   CENTER           hold the circle center, heading captured on entry
 *   HEADING          hold the circle center, heading slewed to circle start tangent
 *   GOTO_START       hold the circle start point
 *   START            fly the circle; hands over to END after t_end
 *   END              hold the circle end position and final heading
 *   STOP             heading is released back to the guidance
 *
 *
 * Every active mode runs the full feedback cascade
 *   v_cmd = V_ref + Kp * (P_ref - P)
 *   a_sp  = A_ref + Kv * (v_cmd - V)        (bounded)
 * and publishes a_sp as an ACCEL_SP ABI message consumed by
 * guidance_indi_hybrid, with the heading commanded through
 * guidance_indi_hybrid_set_heading_sp() / _set_heading_rate_ff().
 *
 * With air_track on, START points the nose along the velocity relative to
 * a wind fitted on the pitot while circling, instead of the circle tangent.
 */

#include "modules/ctrl/circ_trajectory.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
#include "modules/nav/waypoints.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "math/pprz_algebra_float.h"

#include <math.h>

/* ABI sender id for the velocity setpoint */
#ifndef CIRC_TRAJECTORY_VEL_SP_ID
#define CIRC_TRAJECTORY_VEL_SP_ID VEL_SP_CIRC_ID
#endif

/* ABI sender id for the acceleration setpoint */
#ifndef CIRC_TRAJECTORY_ACCEL_SP_ID
#define CIRC_TRAJECTORY_ACCEL_SP_ID ACCEL_SP_CIRC_ID
#endif

#ifndef CIRC_TRAJECTORY_RADIUS
#define CIRC_TRAJECTORY_RADIUS 2.0f
#endif
#ifndef CIRC_TRAJECTORY_V_MAX
#define CIRC_TRAJECTORY_V_MAX 1.0f
#endif
#ifndef CIRC_TRAJECTORY_K
#define CIRC_TRAJECTORY_K 0.2f
#endif
#ifndef CIRC_TRAJECTORY_T_END
#define CIRC_TRAJECTORY_T_END 60.0f
#endif
#ifndef CIRC_TRAJECTORY_PLANE_TILT_DEG
#define CIRC_TRAJECTORY_PLANE_TILT_DEG 0.0f
#endif
#ifndef CIRC_TRAJECTORY_PLANE_AZIMUTH_DEG
#define CIRC_TRAJECTORY_PLANE_AZIMUTH_DEG 0.0f
#endif
#ifndef CIRC_TRAJECTORY_CENTER
#define CIRC_TRAJECTORY_CENTER {0.0f, 0.0f, -1.5f}   // NED [m]
#endif

/* Symmetric bound on the commanded acceleration [m/s^2] */
#ifndef CIRC_TRAJECTORY_MAX_ACCEL
#define CIRC_TRAJECTORY_MAX_ACCEL 3.0f
#endif
#ifndef CIRC_TRAJECTORY_MAX_ACCELZ
#define CIRC_TRAJECTORY_MAX_ACCELZ 4.0f
#endif
#ifndef CIRC_TRAJECTORY_YAW_RATE
#define CIRC_TRAJECTORY_YAW_RATE 10.0f
#endif

/* Heading-rate feedforward gain (1 = feed the full trajectory heading rate) */
#ifndef CIRC_TRAJECTORY_YAW_FF_GAIN
#define CIRC_TRAJECTORY_YAW_FF_GAIN 1.0f
#endif

/* Natural frequency [rad/s] of the critically-damped (zeta = 1) reference model */
#ifndef CIRC_TRAJECTORY_SMOOTH_W
#define CIRC_TRAJECTORY_SMOOTH_W 1.0f
#endif

#ifndef CIRC_TRAJECTORY_ACCEL_LEAD_TIME
#define CIRC_TRAJECTORY_ACCEL_LEAD_TIME 0.0f
#endif

/* Nose on the air-relative velocity in the circle (and pitch preference 10 deg in the guidance) */
#ifndef CIRC_TRAJECTORY_AIR_TRACK
#define CIRC_TRAJECTORY_AIR_TRACK FALSE
#endif

/* Delay of the velocity and acceleration references [s] (Optitrack: 0.0 - GPS: 0.07) */
static const float circ_ref_delay = 0.07f;

/* Horizontal acceleration cap of the GOTO_START transfer [m/s^2] */
static const float circ_goto_max_accel = 5.0f;

/* Pitot wind fit, pitot = k * u_g - (k * wind) . x_body, recursive least squares while circling */
static const float wind_fit_dt = 0.1f;        /* [s] */
static const float wind_fit_lambda = 0.995f;  /* forgetting factor, ~20 s memory */
static const float wind_fit_v_min = 5.0f;     /* forward ground speed below which the pitot is not fitted [m/s] */
static const float wind_fit_lag_sd = 0.5f;    /* pitot lag uncertainty [s] (0.24-0.50 s fitted on 28 Oct) */
static const float wind_fit_noise = 0.5f;     /* pitot residual [m/s] (0.41-0.77 on 28 Oct) */
static const float wind_max = 5.0f;           /* [m/s] */
#define WIND_FIT_LAG 4                        /* pitot lag behind the EKF, in fit steps (0.4 s) */

/* Air track faded in over this speed band [m/s] */
static const float air_track_v_on = 4.0f;
static const float air_track_v_full = 6.0f;
static const float air_track_ramp_t = 2.0f;   /* switch ramp [s] */

static struct {
  float th[3];                 /* k, k * wind_n, k * wind_e */
  float P[3][3];
  float lag[WIND_FIT_LAG][3];  /* u_g, x_body n, x_body e */
  uint8_t lag_idx;
  struct FloatVect2 wind;      /* fitted wind, NED [m/s] */
  float yaw_offset;            /* air track minus tangent heading [rad] */
} circ_wind;

struct CircTraj circ_traj = {
  .r                 = CIRC_TRAJECTORY_RADIUS,
  .v_max             = CIRC_TRAJECTORY_V_MAX,
  .k                 = CIRC_TRAJECTORY_K,
  .t_end             = CIRC_TRAJECTORY_T_END,
  .plane_tilt_deg    = CIRC_TRAJECTORY_PLANE_TILT_DEG,
  .plane_azimuth_deg = CIRC_TRAJECTORY_PLANE_AZIMUTH_DEG,
  .center            = CIRC_TRAJECTORY_CENTER,
  .max_accel         = CIRC_TRAJECTORY_MAX_ACCEL,
  .max_accelz        = CIRC_TRAJECTORY_MAX_ACCELZ,
  .yaw_rate          = CIRC_TRAJECTORY_YAW_RATE,
  .yaw_ff            = CIRC_TRAJECTORY_YAW_FF_GAIN,
  .smooth_w          = CIRC_TRAJECTORY_SMOOTH_W,
  .accel_lead        = CIRC_TRAJECTORY_ACCEL_LEAD_TIME,
  .air_track         = CIRC_TRAJECTORY_AIR_TRACK,
  .status            = CIRC_TRAJ_HOVER,
  .t                 = 0.0f,
  .air_track_w       = CIRC_TRAJECTORY_AIR_TRACK,
};

float circ_trajectory_periodic_freq = CIRC_TRAJECTORY_RUN_FREQ;     /* Module frequency [Hz] */

/**
 * Apply the circle-plane rotation R = Rz(azimuth) * Ry(tilt) to a vector.
 *
 *   R = | cz*cy   -sz   cz*sy |
 *       | sz*cy    cz   sz*sy |
 *       | -sy       0   cy    |
 */
static void circ_apply_rotation(struct FloatVect3 *out, const struct FloatVect3 *in,
                                float cy, float sy, float cz, float sz)
{
  out->x = (cz * cy) * in->x + (-sz) * in->y + (cz * sy) * in->z;
  out->y = (sz * cy) * in->x + (cz) * in->y + (sz * sy) * in->z;
  out->z = (-sy) * in->x + (cy) * in->z;
}

/**
 * Compute the (smoothed) angle and its derivatives at time t.
 *
 */
static void circ_compute_theta(float t, float *theta, float *theta_dot, float *theta_ddot)
{
  const float k = circ_traj.k;
  const float omega = circ_traj.v_max / circ_traj.r;
  float t_eff, dt_eff, ddt_eff;

  if (circ_traj.t_end > 0.f) {
    Bound(t, 0.f, circ_traj.t_end);
    const float T = circ_traj.t_end;
    const float e_end = expf(-k * T);
    const float e_up  = expf(-k * t);
    const float e_dn  = expf(-k * (T - t));

    t_eff   = t * (1.f + e_end) + (e_up - e_dn - 1.f + e_end) / k;
    dt_eff  = (1.f - e_up) * (1.f - e_dn);
    ddt_eff = k * e_up * (1.f - e_dn) - k * e_dn * (1.f - e_up);
  } else {
    // continuous ramp-up only
    const float e = expf(-k * t);
    t_eff   = t + (e - 1.f) / k;
    dt_eff  = 1.f - e;
    ddt_eff = k * e;
  }

  *theta      = omega * t_eff;
  *theta_dot  = omega * dt_eff;
  *theta_ddot = omega * ddt_eff;
}

/**
 * Evaluate the position, velocity, acceleartion setpoints of the
 * circular trajectory in NED frame at trajectory time t, together
 * with the tangent heading and its rate.
 */
static void circ_eval(float t, struct FloatVect3 *p_ref, struct FloatVect3 *v_ref,
                      struct FloatVect3 *a_ref, float *yaw_ref, float *yaw_rate_ref)
{
  float theta, theta_dot, theta_ddot;
  circ_compute_theta(t, &theta, &theta_dot, &theta_ddot);

  const float r = circ_traj.r;
  const float ct = cosf(theta);
  const float st = sinf(theta);
  const float wd = theta_dot;
  const float wdd = theta_ddot;

  // Local-frame reference (circle in the local XY-plane)
  struct FloatVect3 p_loc = { r * ct, r * st, 0.f };
  struct FloatVect3 v_loc = { -r * st * wd, r * ct * wd, 0.f };
  struct FloatVect3 a_loc = {
    -r * (ct * wd * wd + st * wdd),
    -r * (st * wd * wd - ct * wdd),
    0.f
  };
  // Unit tangent (used for heading) and its time derivative
  struct FloatVect3 tan_loc = { -st, ct, 0.f };
  struct FloatVect3 tandot_loc = { -ct * wd, -st * wd, 0.f };

  // Rotation into NED
  const float cy = cosf(RadOfDeg(circ_traj.plane_tilt_deg));
  const float sy = sinf(RadOfDeg(circ_traj.plane_tilt_deg));
  const float cz = cosf(RadOfDeg(circ_traj.plane_azimuth_deg));
  const float sz = sinf(RadOfDeg(circ_traj.plane_azimuth_deg));

  struct FloatVect3 p_rot, tan_rot, tandot_rot;
  circ_apply_rotation(&p_rot, &p_loc, cy, sy, cz, sz);
  circ_apply_rotation(v_ref, &v_loc, cy, sy, cz, sz);
  circ_apply_rotation(a_ref, &a_loc, cy, sy, cz, sz);
  circ_apply_rotation(&tan_rot, &tan_loc, cy, sy, cz, sz);
  circ_apply_rotation(&tandot_rot, &tandot_loc, cy, sy, cz, sz);

  // Translate position by the (fixed) circle center
  VECT3_SUM(*p_ref, p_rot, circ_traj.center);

  // Heading from the tangent direction in the NED horizontal plane, and its
  // rate: d/dt atan2(ty, tx) = (tx*ty_dot - ty*tx_dot) / (tx^2 + ty^2)
  *yaw_ref = atan2f(tan_rot.y, tan_rot.x);
  const float nxy2 = tan_rot.x * tan_rot.x + tan_rot.y * tan_rot.y;
  *yaw_rate_ref = (nxy2 > 1e-6f) ?
                  (tan_rot.x * tandot_rot.y - tan_rot.y * tandot_rot.x) / nxy2 : 0.f;
}

/** Move a flight plan waypoint to a NED position of the circle. */
static void circ_set_wp_ned(uint8_t wp_id, struct FloatVect3 *ned)
{
  struct EnuCoor_f enu = { .x = ned->y, .y = ned->x, .z = -ned->z };
  waypoint_set_enu(wp_id, &enu);
}

float circ_trajectory_height(void)
{
  return -circ_traj.center.z;
}

void circ_trajectory_set_wp_center(uint8_t wp_id)
{
  circ_set_wp_ned(wp_id, &circ_traj.center);
}

void circ_trajectory_set_wp_start(uint8_t wp_id)
{
  struct FloatVect3 p, v, a;
  float yaw, yaw_rate;
  circ_eval(0.f, &p, &v, &a, &yaw, &yaw_rate);
  circ_set_wp_ned(wp_id, &p);
}

void circ_trajectory_set_wp_end(uint8_t wp_id)
{
  struct FloatVect3 p, v, a;
  float yaw, yaw_rate;
  circ_eval(circ_traj.t_end, &p, &v, &a, &yaw, &yaw_rate);
  circ_set_wp_ned(wp_id, &p);
}

static void circ_wind_reset(void)
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      circ_wind.P[i][j] = 0.f;
    }
  }
  circ_wind.th[0] = 1.f;
  circ_wind.th[1] = 0.f;
  circ_wind.th[2] = 0.f;
  circ_wind.P[0][0] = 0.01f;
  circ_wind.P[1][1] = 4.f;
  circ_wind.P[2][2] = 4.f;
  circ_wind.wind.x = 0.f;
  circ_wind.wind.y = 0.f;
}

/* One fit step: the EKF side always goes through the lag line, the pitot is fitted only when asked */
static void circ_wind_step(bool fit)
{
  const struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  const struct NedCoor_f *vel = stateGetSpeedNed_f();

  float *s = circ_wind.lag[circ_wind.lag_idx];
  const float phi[3] = { s[0], -s[1], -s[2] };
  s[0] = RMAT_ELMT(*ned_to_body, 0, 0) * vel->x
         + RMAT_ELMT(*ned_to_body, 0, 1) * vel->y
         + RMAT_ELMT(*ned_to_body, 0, 2) * vel->z;
  s[1] = RMAT_ELMT(*ned_to_body, 0, 0);
  s[2] = RMAT_ELMT(*ned_to_body, 0, 1);
  circ_wind.lag_idx = (circ_wind.lag_idx + 1) % WIND_FIT_LAG;

  const float pitot = stateGetAirspeed_f();
  if (!fit || !(phi[0] > wind_fit_v_min) || !(pitot > 1.f)) {
    return;
  }

  // Down-weighted while the speed changes, where a lag error reads as wind
  const float acc = (s[0] - phi[0]) / (WIND_FIT_LAG * wind_fit_dt) * wind_fit_lag_sd / wind_fit_noise;
  float Pphi[3];
  float den = wind_fit_lambda * (1.f + acc * acc);
  float err = pitot;
  for (int i = 0; i < 3; i++) {
    Pphi[i] = circ_wind.P[i][0] * phi[0] + circ_wind.P[i][1] * phi[1] + circ_wind.P[i][2] * phi[2];
    den += phi[i] * Pphi[i];
    err -= phi[i] * circ_wind.th[i];
  }
  for (int i = 0; i < 3; i++) {
    circ_wind.th[i] += Pphi[i] / den * err;
    for (int j = i; j < 3; j++) {
      circ_wind.P[i][j] = (circ_wind.P[i][j] - Pphi[i] * Pphi[j] / den) / wind_fit_lambda;
      circ_wind.P[j][i] = circ_wind.P[i][j];
    }
  }

  // A pitot scale this far off means the fit went wrong
  if (!(circ_wind.th[0] > 0.5f && circ_wind.th[0] < 1.5f)) {
    circ_wind_reset();
    return;
  }
  circ_wind.wind.x = circ_wind.th[1] / circ_wind.th[0];
  circ_wind.wind.y = circ_wind.th[2] / circ_wind.th[0];
  float_vect2_bound_in_2d(&circ_wind.wind, wind_max);
}

/* Nose on the reference velocity relative to the fitted wind, faded in with the speed and over the first lap */
static void circ_air_track(const struct FloatVect3 *v_ref, const struct FloatVect3 *a_ref, float *yaw, float *yaw_rate)
{
  const float rx = v_ref->x - circ_wind.wind.x;
  const float ry = v_ref->y - circ_wind.wind.y;
  const float r2 = rx * rx + ry * ry;

  // Slower of the reference and its air speed: 0 at both ends of the circle
  const float s = Min(sqrtf(v_ref->x * v_ref->x + v_ref->y * v_ref->y), sqrtf(r2));
  float f = (s - air_track_v_on) / (air_track_v_full - air_track_v_on);
  Bound(f, 0.f, 1.f);
  // The wind fit needs about one lap of heading change
  float theta, theta_dot, theta_ddot;
  circ_compute_theta(circ_traj.t, &theta, &theta_dot, &theta_ddot);
  float lap = theta / (2.f * (float)M_PI);
  Bound(lap, 0.f, 1.f);
  f *= lap * lap * (3.f - 2.f * lap) * circ_traj.air_track_w;
  if (f <= 0.f) {
    return;
  }

  float dpsi = atan2f(ry, rx) - *yaw;
  FLOAT_ANGLE_NORMALIZE(dpsi);
  *yaw += f * dpsi;
  FLOAT_ANGLE_NORMALIZE(*yaw);
  *yaw_rate += f * ((rx * a_ref->y - ry * a_ref->x) / r2 - *yaw_rate);
  circ_wind.yaw_offset = f * dpsi;
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_circ_trajectory(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t)circ_traj.status;
  pprz_msg_send_CIRC_TRAJECTORY(trans, dev, AC_ID, &status, &circ_traj.t);
}

// Wind north, east [m/s], pitot scale, air track heading offset [deg]
static void send_circ_wind(struct transport_tx *trans, struct link_device *dev)
{
  float v[4] = { circ_wind.wind.x, circ_wind.wind.y, circ_wind.th[0], DegOfRad(circ_wind.yaw_offset) };
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, 4, v);
}
#endif

void circ_trajectory_init(void)
{
  circ_wind_reset();
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRC_TRAJECTORY, send_circ_trajectory);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_circ_wind);
#endif
}

void circ_trajectory_start(void)
{
  circ_traj.t = 0.f;
  circ_traj.status = CIRC_TRAJ_START;
}

void circ_trajectory_hover_here(void)
{
  circ_traj.status = CIRC_TRAJ_HOVER;
}

void circ_trajectory_stop(void)
{
  circ_traj.status = CIRC_TRAJ_STOP;
}

/**
 *   Critically damped response to improve tracking performance
 *   a = w^2 * (target - ref_pos) - 2*w * ref_vel      (zeta = 1)
 */
static void circ_smooth_step(struct FloatVect3 *ref_pos, struct FloatVect3 *ref_vel,
                             const struct FloatVect3 *target, float w, float max_accel, float dt,
                             struct FloatVect3 *p_ref, struct FloatVect3 *v_ref,
                             struct FloatVect3 *a_ref)
{
  const float w2 = w * w;
  const float two_w = 2.f * w;

  struct FloatVect3 err;
  VECT3_DIFF(err, *target, *ref_pos);

  if (w2 > 1e-6f) {
    const float d_max_h = max_accel / w2;
    const float d_max_z = circ_traj.max_accelz / w2;
    const float d_h = sqrtf(err.x * err.x + err.y * err.y);
    if (d_h > d_max_h) {
      const float scale = d_max_h / d_h;
      err.x *= scale;
      err.y *= scale;
    }
    Bound(err.z, -d_max_z, d_max_z);
  }

  // model acceleration for this step
  a_ref->x = w2 * err.x - two_w * ref_vel->x;
  a_ref->y = w2 * err.y - two_w * ref_vel->y;
  a_ref->z = w2 * err.z - two_w * ref_vel->z;
  // semi-implicit Euler integration (unconditionally stable for w*dt << 1)
  ref_vel->x += a_ref->x * dt;
  ref_vel->y += a_ref->y * dt;
  ref_vel->z += a_ref->z * dt;
  ref_pos->x += ref_vel->x * dt;
  ref_pos->y += ref_vel->y * dt;
  ref_pos->z += ref_vel->z * dt;
  VECT3_COPY(*p_ref, *ref_pos);
  VECT3_COPY(*v_ref, *ref_vel);
}

void circ_trajectory_run(void)
{
  static enum CircTrajStatus prev_status = (enum CircTrajStatus) - 1;
  static float heading_cmd = 0.f;   // ramped heading setpoint in HEADING mode [rad]
  static struct FloatVect3 ref_pos = {0.f, 0.f, 0.f};
  static struct FloatVect3 ref_vel = {0.f, 0.f, 0.f};

  const enum CircTrajStatus status = circ_traj.status;
  const bool entered = (status != prev_status);
  prev_status = status;

  const float dt = 1.f / circ_trajectory_periodic_freq;

  // Pitot wind fit at 10 Hz, fitted while flying the circle
  static uint16_t wind_cnt = 0;
  if (++wind_cnt >= (uint16_t)(circ_trajectory_periodic_freq * wind_fit_dt + 0.5f)) {
    wind_cnt = 0;
    circ_wind_step(status == CIRC_TRAJ_START && autopilot_in_flight());
  }
  circ_wind.yaw_offset = 0.f;

  // Ramped so that toggling the switch in the circle does not step the heading or the pitch preference
  float dw = (circ_traj.air_track ? 1.f : 0.f) - circ_traj.air_track_w;
  Bound(dw, -dt / air_track_ramp_t, dt / air_track_ramp_t);
  circ_traj.air_track_w += dw;

  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();

  if (status == CIRC_TRAJ_STOP) {
    // Hand the heading back; the vehicle's mixing captures and holds it from here
    if (entered) { guidance_indi_hybrid_release_heading_sp(); }
    VECT3_COPY(ref_pos, *pos);
    VECT3_COPY(ref_vel, *vel);
    return;
  }

  // Guidance consumes our ACCEL_SP only in an autonomous, airborne mode. Until
  // then the smoothing reference is kept on the vehicle (see below).
  const uint8_t ap_mode = autopilot_get_mode();
  const bool guidance_active = autopilot_in_flight() &&
                               (ap_mode == AP_MODE_GUIDED || ap_mode == AP_MODE_NAV);

  struct FloatVect3 p_ref, v_ref, a_ref;
  struct FloatVect3 target;   // fixed hold point for the smoothed modes [NED]
  bool hold_mode = true;      // false only for CIRC_TRAJ_START (direct trajectory)
  float yaw_ref;
  float yaw_rate_ref = 0.f;  // heading-rate feedforward [rad/s], 0 in the hold modes

  switch (status) {

    case CIRC_TRAJ_HOVER: {
      // Hold "here". The pose is captured on entry
      if (entered || !guidance_active) {
        VECT3_COPY(circ_traj.hold_pos, *pos);
        circ_traj.hold_yaw = guidance_indi_hybrid_get_heading();
      }
      VECT3_COPY(target, circ_traj.hold_pos);
      yaw_ref = circ_traj.hold_yaw;
      break;
    }

    case CIRC_TRAJ_CENTER: {
      // Hold the circle center, keeping the heading captured on entry
      if (entered || !guidance_active) { circ_traj.hold_yaw = guidance_indi_hybrid_get_heading(); }
      VECT3_COPY(target, circ_traj.center);
      yaw_ref = circ_traj.hold_yaw;
      break;
    }

    case CIRC_TRAJ_HEADING: {
      // Rotate in place at the circle center, ramping the heading smoothly
      // toward the start tangent at circ_traj.yaw_rate
      struct FloatVect3 ps, vs, as;
      float target_yaw, target_yaw_rate;
      circ_eval(0.f, &ps, &vs, &as, &target_yaw, &target_yaw_rate);
      if (entered || !guidance_active) { heading_cmd = guidance_indi_hybrid_get_heading(); }
      VECT3_COPY(target, circ_traj.center);

      // Bound yaw command by yaw_rate * dt per step
      // to prevent abrupt change in heading
      float dpsi = target_yaw - heading_cmd;
      FLOAT_ANGLE_NORMALIZE(dpsi);
      const float max_step = RadOfDeg(circ_traj.yaw_rate) * dt;
      Bound(dpsi, -max_step, max_step);
      heading_cmd += dpsi;
      FLOAT_ANGLE_NORMALIZE(heading_cmd);
      yaw_ref = heading_cmd;
      yaw_rate_ref = dpsi / dt;  // rate of the ramp itself
      break;
    }

    case CIRC_TRAJ_GOTO_START: {
      // Hold the circle start position (t=0) with the tangent heading.
      struct FloatVect3 vs, as;
      float unused_rate;
      circ_eval(0.f, &target, &vs, &as, &yaw_ref, &unused_rate);
      break;
    }

    case CIRC_TRAJ_START: {
      // Fly the circular trajectory (full P/V/A feedforward + tangent heading + heading rate)
      hold_mode = false;
      if (entered) { circ_traj.t = 0.f; }
      circ_eval(circ_traj.t, &p_ref, &v_ref, &a_ref, &yaw_ref, &yaw_rate_ref);
      if (circ_traj.air_track_w > 0.f) {
        circ_air_track(&v_ref, &a_ref, &yaw_ref, &yaw_rate_ref);
      }
      // v_ref and a_ref run late by the estimator lag
      struct FloatVect3 p_l, v_l;
      float yaw_l, yaw_rate_l;
      const float t_d = Max(circ_traj.t - circ_ref_delay, 0.f);
      circ_eval(t_d, &p_l, &v_ref, &a_ref, &yaw_l, &yaw_rate_l);
      if (circ_traj.accel_lead > 0.f) {
        // a_ref only, evaluated at t + lead to offset the attitude realization lag
        circ_eval(t_d + circ_traj.accel_lead, &p_l, &v_l, &a_ref, &yaw_l, &yaw_rate_l);
      }
      circ_traj.t += dt;

      // Hold the final pose once t_end elapses
      if (circ_traj.t_end > 0.f && circ_traj.t >= circ_traj.t_end) {
        circ_traj.status = CIRC_TRAJ_END;
      }
      break;
    }

    case CIRC_TRAJ_END: {
      // Hold the circle end position and final heading through the full
      // P/V/A feedback cascade
      if (entered) {
        struct FloatVect3 vf, af;
        float rf;
        circ_eval(circ_traj.t_end, &circ_traj.final_pos, &vf, &af, &circ_traj.final_yaw, &rf);
      }
      VECT3_COPY(target, circ_traj.final_pos);
      yaw_ref = circ_traj.final_yaw;
      break;
    }

    default:
      VECT3_COPY(ref_pos, *pos);
      VECT3_COPY(ref_vel, *vel);
      return;
  }

  const bool goto_start = (status == CIRC_TRAJ_GOTO_START);
  const float max_accel = goto_start ? Min(circ_traj.max_accel, circ_goto_max_accel) : circ_traj.max_accel;

  if (hold_mode && guidance_active) {
    if (entered) {
      VECT3_COPY(ref_pos, *pos);
      VECT3_COPY(ref_vel, *vel);
    }
    circ_smooth_step(&ref_pos, &ref_vel, &target, circ_traj.smooth_w, max_accel, dt,
                     &p_ref, &v_ref, &a_ref);
  } else {
    VECT3_COPY(ref_pos, *pos);
    VECT3_COPY(ref_vel, *vel);
    if (hold_mode) {
      VECT3_COPY(p_ref, *pos);
      VECT3_COPY(v_ref, *vel);
      FLOAT_VECT3_ZERO(a_ref);
    }
  }

  struct FloatVect3 p_err;
  p_err.x = p_ref.x - pos->x;
  p_err.y = p_ref.y - pos->y;
  p_err.z = p_ref.z - pos->z;

  // Outer loop velocity setpoint using guidance_indi gains
  struct FloatVect3 v_cmd;
  v_cmd.x = v_ref.x + gih_params.pos_gain  * p_err.x;
  v_cmd.y = v_ref.y + gih_params.pos_gain  * p_err.y;
  v_cmd.z = v_ref.z + gih_params.pos_gainz * p_err.z;

  struct FloatVect3 accel_sp;
  accel_sp.x = a_ref.x + gih_params.speed_gain  * (v_cmd.x - vel->x);
  accel_sp.y = a_ref.y + gih_params.speed_gain  * (v_cmd.y - vel->y);
  accel_sp.z = a_ref.z + gih_params.speed_gainz * (v_cmd.z - vel->z);


  if (goto_start) {
    float_vect3_bound_in_2d(&accel_sp, max_accel);
  } else {
    BoundAbs(accel_sp.x, max_accel);
    BoundAbs(accel_sp.y, max_accel);
  }
  BoundAbs(accel_sp.z, circ_traj.max_accelz);

  guidance_indi_hybrid_set_heading_sp(yaw_ref);
  guidance_indi_hybrid_set_heading_rate_ff(circ_traj.yaw_ff * yaw_rate_ref);

  // Publish the 3D velocity setpoint to guidance_indi_hybrid
  // AbiSendMsgVEL_SP(CIRC_TRAJECTORY_VEL_SP_ID, &v_cmd);
  // Publish the 3D acceleration setpoint to guidance_indi_hybrid
  AbiSendMsgACCEL_SP(CIRC_TRAJECTORY_ACCEL_SP_ID, 1, &accel_sp);
}
