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

/** @file "modules/ctrl/eff_scheduling_atlas.h"
 * Control effectiveness scheduler for the Atlas tiltrotor with
 * two tilt banks (right/left) each carrying 2 rotor in a quad-X layout
 * Body axes: x - forward, y - right, z - down
 */

#ifndef CTRL_EFF_SCHED_ATLAS_H
#define CTRL_EFF_SCHED_ATLAS_H

#include "std.h"

/* Actuator indices for INDI vector (INDI_NUM_ACT = 6/8)*/
#define ATLAS_ACT_MOTOR_FR   0          // Motor Front-Right (FR)
#define ATLAS_ACT_MOTOR_BR   1          // Motor Back-Right (BR)
#define ATLAS_ACT_MOTOR_BL   2          // Motor Back-Left (BL)
#define ATLAS_ACT_MOTOR_FL   3          // Motor Front-Left (FL)
#define ATLAS_ACT_TILT_R     4          // Tilt Servo Right (TILT_R), 0 = vertical, pi/2 = forward
#define ATLAS_ACT_TILT_L     5          // Tilt Servo Left (TILT_L), 0 = vertical, pi/2 = forward
// #define ATLAS_ACT_ELEVON_R   6
// #define ATLAS_ACT_ELEVON_L   7

/* Virtual control vector indices */
#define ATLAS_VC_MX    0                // Angular Acceleration along body x-axis (roll)
#define ATLAS_VC_MY    1                // Angular Acceleration along body y-axis (pitch)
#define ATLAS_VC_MZ    2                // Angular Acceleration along body z-axis (yaw)
#define ATLAS_VC_AZ    3                // Linear Acceleration along body z-axis (vertical)
#define ATLAS_VC_AX    4                // Linear Acceleration along body x-axis (forward)


struct atlas_eff_sched_param_t {
    // Mass/Inertia Constants
    float Ixx;                          // MMOI about longitudinal axis [kgm^2]
    float Iyy;                          // MMOI about lateral axis [kgm^2]
    float Izz;                          // MMOI about vertical axis [kgm^2]
    float m;                            // mass [kg]

    /* Rotor Geometry in body frame [m]
    Order follows rotor convention (FR, BR, BL, FL) */
    float r_x[4];                       // Longitudinal Offset from CG (positive = forward)
    float r_y[4];                       // Lateral Offset from CG (positive = right)
    float r_z[4];                       // Vertical Offset from CG (positive = down)

    /* Rotor Coefficients
    * Quadratic Thrust Model per motor:  T(pprz) = a0 + a1 * pprz + a2 * pprz^2
    * Linearized: dT/dpprz = a0 + 2 * a1 * pprz
    */
    float k_dT_dpprz[2];                // Mapping motor command [pprz] -> thrust [N],
                                        // dT/dpprz = a0 + 2 * a1 * pprz, (k_dT_dpprz = [a0, a1])
    float kappa;                        // Propeller torque coefficient: M = κ * T [m]
    float spin_dir[4];                  // Rotor spin direction (+1 for CW. -1 for CCW)

    // Tilt Coefficients
    float k_tilt_deflect[2];            // Mapping tilt-servo command [pprz] -> tilt angle [rad],
                                        // alpha = b0 + b1 * pprz, (k_tilt_deflect = [b0, b1]

    // Wing Coefficients
    float k_lift;                       // Wing Lift Coefficient: L = k_lift * V^2 / m
    float v_wing;                       // Airspeed to fully thrust wing lift [m/s]

    // // Elevon Coefficients
    // float k_elevon_deflect[2];          // delta = c0 + c1 * pprz (k_elevon_deflection = [c0, c1]
    // float k_elevon_roll;                // dMz_dDelta = k_roll * V^2
    // float k_elevon_pitch;               // dMy_dDelta = k_pitch * V^2
    // float k_elevon_propwash;            // dM_dDelta = k_propwash * T_x * V
};

struct atlas_eff_sched_var_t
{
    // Tilt states, read from ABI callback on tilt-servo feedback
    float alpha_r_rad;                  // Right tilt angle
    float alpha_l_rad;                  // Left tilt angle
    float sin_ar, cos_ar;
    float sin_al, cos_al;

    // Motor states
    float cmd_motor[4];
    float T[4];                         // Thurst per motor [N]
    float dT_dpprz[4];                  // Derivative wrt pprz [N/pprz]

    // Airspeed Measurements
    float airspeed;
    float airspeed_sq;

    // // Elevon Commands
    // float cmd_elevon_r;                 // Right Elevon Command
    // float cmd_elevon_l;                 // Left Elevon Command
};

extern struct atlas_eff_sched_param_t atlas_eff_sched_p;
extern struct atlas_eff_sched_var_t atlas_eff_sched_v;

extern float atlas_eff_liftd;
extern float atlas_eff_tilt_traverse_time;
extern bool  atlas_eff_disable_tilt;       // Debug: freeze tilts at hover (alpha=0)

extern void eff_scheduling_atlas_init(void);
extern void eff_scheduling_atlas_periodic(void);

#endif  // CTRL_EFF_SCHED_ATLAS_H