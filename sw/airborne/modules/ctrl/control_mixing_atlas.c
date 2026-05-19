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
#include "firmwares/rotorcraft/navigation.h"

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

void control_mixing_atlas_init(void)
{
}

void control_mixing_atlas_manual(void)
{
  int16_t throttle = radio_control_get(RADIO_THROTTLE);

  commands[COMMAND_MOTOR_FR] = throttle;
  commands[COMMAND_MOTOR_BR] = throttle;
  commands[COMMAND_MOTOR_BL] = throttle;
  commands[COMMAND_MOTOR_FL] = throttle;

  commands[COMMAND_ROLL]  = radio_control_get(RADIO_ROLL);
  commands[COMMAND_PITCH] = radio_control_get(RADIO_PITCH);
  commands[COMMAND_YAW]   = radio_control_get(RADIO_YAW);
  commands[COMMAND_THRUST] = throttle;
  commands[COMMAND_THRUST_X] = radio_control_get(RADIO_TILT);
  autopilot.throttle       = throttle;
}

void control_mixing_atlas_attitude_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_atlas_attitude(void)
{
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);
  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = stabilization.cmd[COMMAND_THRUST];
}

void control_mixing_atlas_quad_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  actuators_pprz[CMA_ACT_TILT_R] = -MAX_PPRZ;
  actuators_pprz[CMA_ACT_TILT_L] = -MAX_PPRZ;
}

void control_mixing_atlas_quad(void)
{
  int16_t throttle = radio_control_get(RADIO_THROTTLE);

  commands[COMMAND_MOTOR_FR] = throttle;
  commands[COMMAND_MOTOR_BR] = throttle;
  commands[COMMAND_MOTOR_BL] = throttle;
  commands[COMMAND_MOTOR_FL] = throttle;

  commands[COMMAND_ROLL]    = radio_control_get(RADIO_ROLL);
  commands[COMMAND_PITCH]   = radio_control_get(RADIO_PITCH);
  commands[COMMAND_YAW]     = radio_control_get(RADIO_YAW);
  commands[COMMAND_THRUST]  = throttle;
  commands[COMMAND_THRUST_X] = 0;
  actuators_pprz[CMA_ACT_TILT_R] = -MAX_PPRZ;
  actuators_pprz[CMA_ACT_TILT_L] = -MAX_PPRZ;
  autopilot.throttle = throttle;
}

// void control_mixing_atlas_nav_enter(void)
// {
//   guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
//   guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
//   stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
// }
//
// void control_mixing_atlas_nav_run(void)
// {
//   struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
//   struct StabilizationSetpoint stab_sp = guidance_h_run(autopilot_in_flight());
//   stabilization_run(autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);
//   SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
//   autopilot.throttle = stabilization.cmd[COMMAND_THRUST];
// }