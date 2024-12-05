/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/ctrl/control_mixing_heewing.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing specific to the Heewing T1 Ranger
 */

#include "modules/ctrl/control_mixing_heewing.h"
#include "modules/radio_control/radio_control.h"
#include "generated/radio.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

// Tilt position in forward flight
#ifndef CMH_TILT_FORWARD
#define CMH_TILT_FORWARD 0
#endif

// Tilt vertical position for hovering
#ifndef CMH_TILT_VERTICAL
#define CMH_TILT_VERTICAL 8700
#endif

// Max tilt differential for yaw
#ifndef CMH_TILT_DIFF_MAX
#define CMH_TILT_DIFF_MAX (MAX_PPRZ-CMH_TILT_VERTICAL)
#endif

// Motor idle position
#ifndef CMH_MOTOR_IDLE
#define CMH_MOTOR_IDLE 800
#endif

void control_mixing_heewing_init(void)
{
  // your init code here
}

void control_mixing_heewing_manual(void)
{
  commands[COMMAND_ROLL] = radio_control_get(RADIO_ROLL);
  commands[COMMAND_PITCH] = radio_control_get(RADIO_PITCH);
  commands[COMMAND_YAW] = 0;
  commands[COMMAND_TILT] = CMH_TILT_FORWARD;
  commands[COMMAND_MOTOR_RIGHT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_LEFT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_TAIL] = MIN_PPRZ;
  commands[COMMAND_THRUST] = (commands[COMMAND_MOTOR_RIGHT] + commands[COMMAND_MOTOR_LEFT]) / 2;
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_heewing_attitude_direct(void)
{
  commands[COMMAND_TILT] = CMH_TILT_VERTICAL;
  commands[COMMAND_ROLL] = 0;
  commands[COMMAND_PITCH] = 0;
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);
  if (autopilot_get_motors_on()) {
    commands[COMMAND_MOTOR_RIGHT] = actuators_pprz[CMH_ACT_MOTOR_RIGHT];
    commands[COMMAND_MOTOR_LEFT]  = actuators_pprz[CMH_ACT_MOTOR_LEFT];
    commands[COMMAND_MOTOR_TAIL]  = actuators_pprz[CMH_ACT_MOTOR_TAIL];
    if (autopilot_in_flight()) {
      commands[COMMAND_YAW]       = actuators_pprz[CMH_ACT_YAW];
    } else {
      commands[CMH_ACT_YAW]      = 0;
    }
    commands[COMMAND_THRUST]      = stabilization.cmd[COMMAND_THRUST];
  } else {
    commands[COMMAND_MOTOR_RIGHT] = -MAX_PPRZ;
    commands[COMMAND_MOTOR_LEFT]  = -MAX_PPRZ;
    commands[COMMAND_MOTOR_TAIL]  = -MAX_PPRZ;
    commands[COMMAND_YAW]         = 0;
    commands[COMMAND_THRUST]      = 0;
  }
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_heewing_attitude_direct_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void stabilization_indi_set_wls_settings(void)
{
   // Calculate the min and max increments
   wls_stab_p.u_min[CMH_ACT_MOTOR_RIGHT] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_RIGHT] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_RIGHT] = act_pref[CMH_ACT_MOTOR_RIGHT];

   wls_stab_p.u_min[CMH_ACT_MOTOR_LEFT] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_LEFT] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_LEFT] = act_pref[CMH_ACT_MOTOR_LEFT];

   wls_stab_p.u_min[CMH_ACT_MOTOR_TAIL] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_TAIL] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_TAIL] = act_pref[CMH_ACT_MOTOR_TAIL];

   wls_stab_p.u_min[CMH_ACT_YAW] = -CMH_TILT_DIFF_MAX;
   wls_stab_p.u_max[CMH_ACT_YAW] = CMH_TILT_DIFF_MAX;
   wls_stab_p.u_pref[CMH_ACT_YAW] = act_pref[CMH_ACT_YAW];
}

