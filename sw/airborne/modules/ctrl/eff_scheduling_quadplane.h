/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger.fr> Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr>
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

/** @file "modules/ctrl/eff_scheduling_quadplane.h"
 *  @brief Interpolation of control effectivenss matrix of a quadplane
 */

#ifndef EFF_SCHEDULING_QUADPLANE_H
#define EFF_SCHEDULING_QUADPLANE_H

#include "std.h"

/* INDI actuator indices */
#define QUADPLANE_ACT_MOTOR_FR    0
#define QUADPLANE_ACT_MOTOR_BR    1
#define QUADPLANE_ACT_MOTOR_BL    2
#define QUADPLANE_ACT_MOTOR_FL    3
#define QUADPLANE_ACT_ELEVON_R    4
#define QUADPLANE_ACT_ELEVON_L    5
#define QUADPLANE_ACT_PUSHER      6

/* Virtual control vector indices */
#define QUADPLANE_VC_MX           0  // Angular Acceleration along body x-axis (roll)
#define QUADPLANE_VC_MY           1  // Angular Acceleration along body y-axis (pitch)
#define QUADPLANE_VC_MZ           2  // Angular Acceleration along body z-axis (yaw)
#define QUADPLANE_VC_AZ           3  // Linear Acceleration along body z-axis (vertical)
#define QUADPLANE_VC_AX           4  // Linear Acceleration along body x-axis (forward)

struct quadplane_eff_sched_param_t {
  float Ixx;                       // MMOI about longitudinal axis [kgm^2]
  float Iyy;                       // MMOI about lateral axis [kgm^2]
  float Izz;                       // MMOI about vertical axis [kgm^2]
  float mass;                      // mass [kg]

  // Rotor Geometry in body frame [m] Order follows rotor convention (FR, BR, BL, FL)
  float r_x[4];                    // Longitudinal Offset from CG (positive = forward)
  float r_y[4];                    // Lateral Offset from CG (positive = right)
  float r_z[4];                    // Vertical Offset from CG (positive = down)
  float spin_dir[4];               // Rotor spin direction (+1 for CW. -1 for CCW)

  float rotor_k_t_pprz[2];         // Quadratic thrust coefficients:
                                    //   [0]: b — linear term [N/pprz]
                                    //   [1]: c — quadratic term [N/pprz^2]
  float kappa;                     // Propeller torque coefficient: M = κ * T [m]

  float pusher_k_t_pprz[2];        // Quadratic thrust coefficients for pusher motor
  float pusher_y;                  // Lateral Offset from CG (positive = right)
  float pusher_z;                  // Vertical Offset from CG (positive = down)
  float pusher_spin_dir;           // Pusher spin direction (+1 for CW. -1 for CCW)

  float elevon_deflect;            // Single slope: delta = k_elevon_deflect * pprz [rad/pprz]
  float elevon_roll;               // dMx_dDelta = k_roll * V^2 [N.m/rad per (m/s)^2]
  float elevon_pitch;              // dMy_dDelta = k_pitch * V^2 [N.m/rad per (m/s)^2]
  float elevon_propwash;           // dMy_dDelta += k_propwash * T_x * V (0 @ V = 0)
  float elevon_diff_drag_yaw;      // dMz/dDelta = +/- k_yaw * V^2 * delta [N.m/rad]
  float elevon_v_full;             // Forward speed for full elevon travel [m/s] (0 = always full)

  float k_lift;
  float v_wing;
};

struct quadplane_eff_sched_var_t {
  float rotor_cmd[4];
  float rotor_cmd_T[4];            // Thurst per motor [N]
  float rotor_dT_dpprz[4];         // Derivative wrt pprz [N/pprz]

  float pusher_cmd;
  float pusher_cmd_T;              // Thurst per motor [N]
  float pusher_dT_dpprz;           // Derivative wrt pprz [N/pprz]

  float airspeed;
  float airspeed_sq;

  // Elevon Commands
  float cmd_elevon_r;              // Right Elevon Command
  float cmd_elevon_l;              // Left Elevon Command
};

extern struct quadplane_eff_sched_var_t quadplane_eff_sched_v;
extern struct quadplane_eff_sched_param_t quadplane_eff_sched_p;

void eff_scheduling_quadplane_init(void);
void eff_scheduling_quadplane_periodic(void);
void eff_scheduling_quadplane_report(void);
void stabilization_indi_set_wls_settings(void);

#endif  // EFF_SCHEDULING_QUADPLANE_H
