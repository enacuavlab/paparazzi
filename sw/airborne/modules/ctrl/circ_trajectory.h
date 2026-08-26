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
 * @file modules/ctrl/circ_trajectory.h
 *
 * Circular reference trajectory generator for the INDI hybrid guidance.
 *
 * Generates a smooth position / velocity / acceleration reference along a
 * circular trajectory in the NED frame and feeds it to guidance_indi_hybrid
 * through the VEL_SP ABI message, together with a heading setpoint commanded
 * via guidance_indi_hybrid_set_heading_sp().
 *
 * Calculates the velocity setpoint in a cascaded manner using
 * guidance_indi gains:
 * vel_sp = V_ref + Kp * (P_ref - P)
 */

#ifndef CIRC_TRAJECTORY_H
#define CIRC_TRAJECTORY_H

#include "std.h"
#include "math/pprz_algebra_float.h"

/**
 * Trajectory mode, selected manually from the ground station
 * (default: CIRC_TRAJ_HOVER). The enum is ordered as the mission is flown:
 *   HOVER -> CENTER -> HEADING -> GOTO_START -> START -> END -> CENTER
 * The only automatic transition is START -> END when a finite duration t_end
 * elapses; every other step is an operator selection. All position-hold modes
 * run the full position/velocity/acceleration feedback cascade.
 *
 * HOVER alone holds "here": the pose captured when the mode is entered (and
 * re-captured every cycle for as long as the guidance is not tracking us), so
 * it never commands a translation and is safe to enter from anywhere. CENTER
 * is the first mode that moves the aircraft onto the circle geometry.
 */
enum CircTrajStatus {
  CIRC_TRAJ_STOP = 0,    ///< inactive, nothing is sent to the guidance
  CIRC_TRAJ_HOVER,       ///< default: hold here, pose captured on entry
  CIRC_TRAJ_CENTER,      ///< hold the circle center, heading captured on entry
  CIRC_TRAJ_HEADING,     ///< hold the circle center, heading slewed to the start tangent
  CIRC_TRAJ_GOTO_START,  ///< hold the circle start point with the tangent heading
  CIRC_TRAJ_START,       ///< fly the circle; hands over to END after t_end
  CIRC_TRAJ_END          ///< hold the circle end position and final heading
};

struct CircTraj {
  /* --- fixed trajectory parameters --- */
  float r;                  ///< circle radius [m]
  float v_max;              ///< maximum tangential speed [m/s]; omega = v_max / r
  float k;                  ///< exponential ramp (wind-up / wind-down) factor [1/s]
  float t_end;              ///< total trajectory duration [s]; <= 0 means run continuously
  float plane_tilt_deg;     ///< tilt of the circle plane [deg] (rotation about local Y)
  float plane_azimuth_deg;  ///< azimuth of the circle plane [deg] (rotation about local Z)
  struct FloatVect3 center; ///< circle center in NED [m]
  float max_accel;          ///< bound on the commanded horizontal acceleration [m/s^2]
  float max_accelz;         ///< bound on the commanded vertical acceleration [m/s^2]
  float yaw_rate;           ///< max heading slew rate while aligning (HEADING mode) [deg/s]
  float yaw_ff;             ///< heading-rate feedforward gain (1 = full trajectory heading rate)
  float smooth_w;           ///< natural frequency of the critically-damped reference model easing the hold-point setpoint [rad/s]

  /* --- runtime state --- */
  enum CircTrajStatus status;
  float t;                    ///< elapsed trajectory time [s]
  struct FloatVect3 hold_pos;  ///< "here" position captured by HOVER (NED) [m]
  float hold_yaw;              ///< heading captured by HOVER / CENTER [rad]
  struct FloatVect3 final_pos; ///< circle end position, computed on END entry (NED) [m]
  float final_yaw;             ///< circle end heading, computed on END entry [rad]
};

extern struct CircTraj circ_traj;
extern float circ_trajectory_periodic_freq;

extern void circ_trajectory_init(void);
extern void circ_trajectory_run(void);

/** Start the circular trajectory from t = 0. */
extern void circ_trajectory_start(void);

/** Hold the current position and heading (HOVER "here"). */
extern void circ_trajectory_hover_here(void);

/** Stop the trajectory and release control back to the flight plan. */
extern void circ_trajectory_stop(void);

/** Height of the circle center above the local reference [m] (positive up). */
extern float circ_trajectory_height(void);

/**
 * Move a flight plan waypoint to the circle center / start (t = 0) / end
 * (t = t_end) position, computed from the current circ_traj parameters.
 * Display and nav-fallback only; the trajectory itself is flown from
 * circ_traj directly.
 */
extern void circ_trajectory_set_wp_center(uint8_t wp_id);
extern void circ_trajectory_set_wp_start(uint8_t wp_id);
extern void circ_trajectory_set_wp_end(uint8_t wp_id);

#endif // CIRC_TRAJECTORY_H
