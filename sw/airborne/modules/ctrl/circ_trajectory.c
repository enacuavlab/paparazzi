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
 * The circular motion is smoothed at the start and end (if t_end != 0) with
 * a first-order exponential wind-up/wind-down on the time variable,
 * controlled by k.
 *
 * Modes (selected from the ground station, see enum CircTrajStatus):
 *   HOVER (default)  hold the circle center, heading captured on entry
 *   HEADING          hold the circle center, heading aligned to the circle
 *                    start tangent (rotate in place before translating)
 *   GOTO_START       hold the circle start point, heading aligned to the
 *                    circle tangent
 *   START            fly the circle; auto-transitions to END after t_end
 *   END              hold the circle end position and final heading
 *   STOP             inactive, heading is released back to the guidance
 *
 * Every active mode runs the full feedback cascade
 *   v_cmd = V_ref + Kp * (P_ref - P)
 *   a_sp  = A_ref + Kv * (v_cmd - V)        (bounded)
 * using the guidance_indi gains, and publishes a_sp on the ACCEL_SP ABI
 * message (3D) consumed by guidance_indi_hybrid. The heading setpoint is
 * commanded directly with guidance_indi_hybrid_set_heading_sp().
 */

#include "modules/ctrl/circ_trajectory.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
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

/* --- fixed trajectory parameters  --- */
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
  .status            = CIRC_TRAJ_HOVER,
  .t                 = 0.0f,
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
 * Wind-up (t < t_end/2):   τ = t
 * Wind down (t > t_end/2): τ = T - t
 *
 *   t_eff   = t + (e^{-k * τ} - 1) / k
 *   dt_eff  = 1 - e^{-k * τ}
 *   ddt_eff = k * e^{-k * τ}
 */
static void circ_compute_theta(float t, float *theta, float *theta_dot, float *theta_ddot)
{
  const float k = circ_traj.k;
  const float omega = circ_traj.v_max / circ_traj.r;
  float t_eff, dt_eff, ddt_eff;

  if (circ_traj.t_end > 0.f) {
    Bound(t, 0.f, circ_traj.t_end);
    const float T = circ_traj.t_end;
    const float t_mid = 0.5f * T;
    const float e_mid = expf(-k * t_mid);
    const float t_eff_mid = t_mid + (e_mid - 1.f) / k;

    if (t <= t_mid) {
      // ramp-up
      const float e = expf(-k * t);
      t_eff   = t + (e - 1.f) / k;
      dt_eff  = 1.f - e;
      ddt_eff = k * e;
    } else {
      // ramp-down (mirror of the ramp-up around t_mid)
      const float tau = T - t;
      const float e = expf(-k * tau);
      const float t_eff_raw = t - (e - 1.f) / k;
      const float t_eff_mid_down = t_mid - (expf(-k * t_mid) - 1.f) / k;
      // enforce continuity at t_mid
      const float shift = t_eff_mid - t_eff_mid_down;
      t_eff   = t_eff_raw + shift;
      dt_eff  = 1.f - e;
      ddt_eff = -k * e;
    }
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
 * circular trajectory in NED frame at trajectory time t.
 */
static void circ_eval(float t, struct FloatVect3 *p_ref, struct FloatVect3 *v_ref,
                      struct FloatVect3 *a_ref, float *yaw_ref)
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
  // Unit tangent (used for heading)
  struct FloatVect3 tan_loc = { -st, ct, 0.f };

  // Rotation into NED
  const float cy = cosf(RadOfDeg(circ_traj.plane_tilt_deg));
  const float sy = sinf(RadOfDeg(circ_traj.plane_tilt_deg));
  const float cz = cosf(RadOfDeg(circ_traj.plane_azimuth_deg));
  const float sz = sinf(RadOfDeg(circ_traj.plane_azimuth_deg));

  struct FloatVect3 p_rot, tan_rot;
  circ_apply_rotation(&p_rot, &p_loc, cy, sy, cz, sz);
  circ_apply_rotation(v_ref, &v_loc, cy, sy, cz, sz);
  circ_apply_rotation(a_ref, &a_loc, cy, sy, cz, sz);
  circ_apply_rotation(&tan_rot, &tan_loc, cy, sy, cz, sz);

  // Translate position by the (fixed) circle center
  VECT3_SUM(*p_ref, p_rot, circ_traj.center);

  // Heading from the tangent direction in the NED horizontal plane
  *yaw_ref = atan2f(tan_rot.y, tan_rot.x);
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_circ_trajectory(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t)circ_traj.status;
  pprz_msg_send_CIRC_TRAJECTORY(trans, dev, AC_ID, &status, &circ_traj.t);
}
#endif

void circ_trajectory_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRC_TRAJECTORY, send_circ_trajectory);
#endif
}

void circ_trajectory_start(void)
{
  circ_traj.t = 0.f;
  circ_traj.status = CIRC_TRAJ_HOVER;
}

void circ_trajectory_stop(void)
{
  circ_traj.status = CIRC_TRAJ_STOP;
}

void circ_trajectory_run(void)
{
  // Forced to an invalid value so the first run triggers the entry action
  // of the boot mode (HOVER captures the heading)
  static enum CircTrajStatus prev_status = (enum CircTrajStatus) - 1;
  static float hold_heading = 0.f;  // heading captured on HOVER entry [rad]
  static float heading_cmd = 0.f;   // ramped heading setpoint in HEADING mode [rad]

  const enum CircTrajStatus status = circ_traj.status;
  const bool entered = (status != prev_status);
  prev_status = status;

  const float dt = 1.f / circ_trajectory_periodic_freq;

  if (status == CIRC_TRAJ_STOP) {
    if (entered) {
      // Release the heading back to the guidance, syncing its integrator to
      // the current heading so the heading setpoint does not snap
      guidance_indi_hybrid_release_heading_sp();
      guidance_indi_hybrid_heading_sp = stateGetNedToBodyEulers_f()->psi;
    }
    return;
  }

  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();

  struct FloatVect3 p_ref, v_ref, a_ref;
  float yaw_ref;

  switch (status) {

    case CIRC_TRAJ_HOVER: {
      if (entered || !autopilot_in_flight()) { hold_heading = stateGetNedToBodyEulers_f()->psi; }
      VECT3_COPY(p_ref, circ_traj.center);
      FLOAT_VECT3_ZERO(v_ref);
      FLOAT_VECT3_ZERO(a_ref);
      yaw_ref = hold_heading;
      break;
    }

    case CIRC_TRAJ_HEADING: {
      // Rotate in place at the circle center, ramping the heading smoothly toward the
      // start tangent at circ_traj.yaw_rate instead of stepping the setpoint.
      float target_yaw;
      circ_eval(0.f, &p_ref, &v_ref, &a_ref, &target_yaw);
      VECT3_COPY(p_ref, circ_traj.center);
      FLOAT_VECT3_ZERO(v_ref);
      FLOAT_VECT3_ZERO(a_ref);

      // Bound yaw command by yaw_rate * dt per step
      // to prevent abrupt change in heading
      if (entered) { heading_cmd = stateGetNedToBodyEulers_f()->psi; }
      float dpsi = target_yaw - heading_cmd;
      FLOAT_ANGLE_NORMALIZE(dpsi);
      const float max_step = RadOfDeg(circ_traj.yaw_rate) * dt;
      Bound(dpsi, -max_step, max_step);
      heading_cmd += dpsi;
      FLOAT_ANGLE_NORMALIZE(heading_cmd);
      yaw_ref = heading_cmd;
      break;
    }

    case CIRC_TRAJ_GOTO_START: {
      // Hold the circle start position (t=0) with the tangent heading.
      circ_eval(0.f, &p_ref, &v_ref, &a_ref, &yaw_ref);
      FLOAT_VECT3_ZERO(v_ref);
      FLOAT_VECT3_ZERO(a_ref);
      break;
    }

    case CIRC_TRAJ_START: {
      // Fly the circular trajectory (full P/V/A feedforward + tangent heading)
      if (entered) { circ_traj.t = 0.f; }
      circ_eval(circ_traj.t, &p_ref, &v_ref, &a_ref, &yaw_ref);
      circ_traj.t += dt;

      // Finite trajectory: hold the final pose once t_end elapses
      if (circ_traj.t_end > 0.f && circ_traj.t >= circ_traj.t_end) {
        circ_traj.status = CIRC_TRAJ_END;
      }
      break;
    }

    case CIRC_TRAJ_END: {
      // Hold the circle end position and final heading through the full
      // P/V/A feedback cascade. The end pose is recomputed on entry so the
      // mode also works when selected manually from the ground station.
      if (entered) {
        struct FloatVect3 vf, af;
        circ_eval(circ_traj.t_end, &circ_traj.final_pos, &vf, &af, &circ_traj.final_yaw);
      }
      VECT3_COPY(p_ref, circ_traj.final_pos);
      FLOAT_VECT3_ZERO(v_ref);
      FLOAT_VECT3_ZERO(a_ref);
      yaw_ref = circ_traj.final_yaw;
      break;
    }

    default:
      return;
  }

  // Outer loop velocity setpoint using guidance_indi gains
  // Horizontal: pos_gain. Vertical: pos_gainz. The inner speed_gain loop is
  // closed inside guidance_indi_hybrid.
  struct FloatVect3 v_cmd;
  v_cmd.x = v_ref.x + gih_params.pos_gain  * (p_ref.x - pos->x);
  v_cmd.y = v_ref.y + gih_params.pos_gain  * (p_ref.y - pos->y);
  v_cmd.z = v_ref.z + gih_params.pos_gainz * (p_ref.z - pos->z);

  struct FloatVect3 accel_sp;
  accel_sp.x = a_ref.x + gih_params.speed_gain  * (v_cmd.x - vel->x);
  accel_sp.y = a_ref.y + gih_params.speed_gain  * (v_cmd.y - vel->y);
  accel_sp.z = a_ref.z + gih_params.speed_gainz * (v_cmd.z - vel->z);


  BoundAbs(accel_sp.x, circ_traj.max_accel);
  BoundAbs(accel_sp.y, circ_traj.max_accel);
  BoundAbs(accel_sp.z, circ_traj.max_accelz);

  guidance_indi_hybrid_set_heading_sp(yaw_ref);


  // Publish the 3D velocity setpoint to guidance_indi_hybrid
  // AbiSendMsgVEL_SP(CIRC_TRAJECTORY_VEL_SP_ID, &v_cmd);
  // Publish the 3D acceleration setpoint to guidance_indi_hybrid
  AbiSendMsgACCEL_SP(CIRC_TRAJECTORY_ACCEL_SP_ID, 1, &accel_sp);
}
