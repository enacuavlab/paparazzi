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

/** @file "modules/ctrl/control_mixing_atlas.c"
 * Control mixing for the Atlas tiltrotor.
 */

#include "modules/ctrl/control_mixing_atlas.h"
#include "modules/ctrl/eff_scheduling_atlas.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

#ifndef GUIDANCE_INDI_MAX_H_THRUST
#define GUIDANCE_INDI_MAX_H_THRUST 0.5f
#endif


struct ThrustSetpoint guidance_set_rc_h_thrust(struct ThrustSetpoint *v_sp)
{
  float thrust[3];
  thrust[0] = GUIDANCE_INDI_MAX_H_THRUST * radio_control_get(RADIO_TILT) / MAX_PPRZ;
  thrust[1] = 0.f;
  thrust[2] = th_sp_to_thrust_f(v_sp, 0, THRUST_AXIS_Z);
  return th_sp_from_thrust_vect_f(thrust);
}

// /* Motor idle command [pprz] */
// #ifndef CMA_MOTOR_IDLE
// #define CMA_MOTOR_IDLE 800
// #endif
//
// void stabilization_indi_set_wls_settings(void)
// {
//   /* Motors: idle floor, MAX_PPRZ ceiling. */
//   for (int i = CMA_ACT_MOTOR_FR; i <= CMA_ACT_MOTOR_FL; i++) {
//     wls_stab_p.u_min[i]  = CMA_MOTOR_IDLE;
//     wls_stab_p.u_max[i]  = MAX_PPRZ;
//     wls_stab_p.u_pref[i] = act_pref[i];
//   }
//   /* Tilt servos: full pprz range. */
//   for (int i = CMA_ACT_TILT_R; i <= CMA_ACT_TILT_L; i++) {
//     wls_stab_p.u_min[i]  = -MAX_PPRZ;
//     wls_stab_p.u_max[i]  = MAX_PPRZ;
//     wls_stab_p.u_pref[i] = act_pref[i];
//   }
// }

static inline void copy_indi_commands(void)
{
  commands[COMMAND_MOTOR_FR] = stabilization.cmd[COMMAND_MOTOR_FR];
  commands[COMMAND_MOTOR_BR] = stabilization.cmd[COMMAND_MOTOR_BR];
  commands[COMMAND_MOTOR_BL] = stabilization.cmd[COMMAND_MOTOR_BL];
  commands[COMMAND_MOTOR_FL] = stabilization.cmd[COMMAND_MOTOR_FL];
  // commands[COMMAND_TILT_F]   = stabilization.cmd[COMMAND_TILT_F];
  // commands[COMMAND_TILT_M]   = stabilization.cmd[COMMAND_TILT_M];
  commands[COMMAND_TILT_R]   = stabilization.cmd[COMMAND_TILT_R];
  commands[COMMAND_TILT_L]   = stabilization.cmd[COMMAND_TILT_L];
  commands[COMMAND_THRUST]   = stabilization.cmd[COMMAND_THRUST];
  autopilot.throttle         = stabilization.cmd[COMMAND_THRUST];
}

void control_mixing_atlas_init(void)
{
}

void control_mixing_atlas_quad_enter(void)
{
  atlas_eff_disable_tilt = true;
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  // actuators_pprz[ATLAS_ACT_TILT_F] = 0.f;   // TILT_F / TILT_M version
  // actuators_pprz[ATLAS_ACT_TILT_M] = 0.f;
  actuators_pprz[ATLAS_ACT_TILT_R] = 0.f;
  actuators_pprz[ATLAS_ACT_TILT_L] = 0.f;
}

void control_mixing_atlas_quad(void)
{
  struct ThrustSetpoint v_sp = guidance_v_run(autopilot_in_flight());
  float thrust_vect[3] = {0.f, 0.f, th_sp_to_thrust_f(&v_sp, 0, THRUST_AXIS_Z)};
  struct ThrustSetpoint th_sp = th_sp_from_thrust_vect_f(thrust_vect);
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);
  copy_indi_commands();
}

void control_mixing_atlas_attitude_enter(void)
{
  atlas_eff_disable_tilt = false;
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_atlas_attitude(void)
{
  struct ThrustSetpoint v_sp = guidance_v_run(autopilot_in_flight());
  struct ThrustSetpoint th_sp = guidance_set_rc_h_thrust(&v_sp);
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);
  copy_indi_commands();
}


struct FloatVect3 atlas_pos_sp  = {0.f, 0.f, -1.f};  // NED position setpoint [m]
struct FloatVect3 atlas_vel_sp  = {0.f, 0.f, 0.f};  // NED velocity setpoint [m/s]
float             atlas_heading_sp = 0.f;           // heading setpoint [rad, NED]

void control_mixing_atlas_guidance_enter(void)
{
  atlas_eff_disable_tilt = false;
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_atlas_guidance(void)
{
  bool in_flight = autopilot_in_flight();

  // Heading setpoint for the guidance loop
  guidance_h.sp.heading = atlas_heading_sp;

  struct StabilizationSetpoint stab_sp = guidance_indi_run_mode(
      in_flight, &guidance_h, &guidance_v,
      GUIDANCE_INDI_HYBRID_H_POS, GUIDANCE_INDI_HYBRID_V_POS);

  // Retrieve the vertical thrust setpoint computed inside run_mode
  struct ThrustSetpoint thrust_sp = guidance_v_run_pos(in_flight, &guidance_v);

  stabilization_run(in_flight, &stab_sp, &thrust_sp, stabilization.cmd);
  copy_indi_commands();
}


void control_mixing_atlas_failsafe(void)
{
  struct StabilizationSetpoint stab_sp = stabilization_get_failsafe_sp();
  struct ThrustSetpoint thrust_sp = guidance_v_run(autopilot_in_flight());
  stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd);
  copy_indi_commands();
}