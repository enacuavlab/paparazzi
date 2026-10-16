/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr> Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr
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

/** @file "modules/ctrl/control_mixing_quadplane.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing for quadplane
 */

#include "modules/ctrl/control_mixing_quadplane.h"
#include "modules/ctrl/eff_scheduling_quadplane.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "autopilot.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/autopilot_utils.h"

float   quadplane_heading_sp = 0.f;           // heading setpoint [rad, NED]

/**
 * Pusher GATE
 *
 * Pusher can be controlled only once the autopilot inflight flag is true. 
 * The autonomous take-off and langing behaves as a pure quad. 
 */
static inline void quadplane_pusher_gate(bool in_flight)
{
  if (!in_flight && !quadplane_eff_disable_pusher) {
    actuators_pprz[QUADPLANE_ACT_PUSHER] = 0;
  }
  quadplane_eff_disable_pusher = !in_flight;
}


void control_mixing_quadplane_init(void)
{

}

/* Attitude direct : Quad only mode in manual for quadplane */
void control_mixing_quadplane_attitude_direct_enter(void)
{
  quadplane_eff_disable_pusher = true;
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  actuators_pprz[QUADPLANE_ACT_PUSHER] = 0.f;
}

void control_mixing_quadplane_attitude_direct(void)
{
  struct ThrustSetpoint v_sp = guidance_v_run(autopilot_in_flight());
  float thrust_vect[3] = {0.f, 0.f, th_sp_to_thrust_f(&v_sp, 0, THRUST_AXIS_Z)};
  struct ThrustSetpoint th_sp = th_sp_from_thrust_vect_f(thrust_vect);
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_quadplane_guidance_enter(void)
{
  quadplane_pusher_gate(autopilot_in_flight());
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_quadplane_guidance(void)
{
  bool in_flight = autopilot_in_flight();
  quadplane_pusher_gate(in_flight);

  if (!in_flight) {
    guidance_h_hover_enter();
  }

  /*struct StabilizationSetpoint stab_sp = guidance_indi_run_mode(
      in_flight, &guidance_h, &guidance_v,
      GUIDANCE_INDI_HYBRID_H_POS, GUIDANCE_INDI_HYBRID_V_POS);*/

  // Retrieve the vertical thrust setpoint computed inside run_mode
  struct ThrustSetpoint thrust_sp = guidance_v_run_pos(in_flight, &guidance_v);
  struct StabilizationSetpoint stab_sp = guidance_h_from_nav(in_flight);

  stabilization_run(in_flight, &stab_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_quadplane_nav_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
  guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_quadplane_nav(void)
{
  bool in_flight = autopilot_in_flight();
  quadplane_pusher_gate(in_flight);

  if (!in_flight) {
    guidance_h_nav_enter();
  }

  /*
  struct StabilizationSetpoint stab_sp = guidance_indi_run_mode(
      in_flight, &guidance_h, &guidance_v,
      GUIDANCE_INDI_HYBRID_H_POS, GUIDANCE_INDI_HYBRID_V_POS);*/

  // Vertical thrust from the flight-plan vertical mode (climb / hover / descend)
  struct ThrustSetpoint thrust_sp = guidance_v_run(in_flight);
  struct StabilizationSetpoint stab_sp = guidance_h_from_nav(in_flight);

  stabilization_run(in_flight, &stab_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}


void control_mixing_quadplane_failsafe(void)
{
  bool in_flight = autopilot_in_flight();
  quadplane_pusher_gate(in_flight);

  struct StabilizationSetpoint stab_sp = stabilization_get_failsafe_sp();
  struct ThrustSetpoint thrust_sp = guidance_v_run(in_flight);
  stabilization_run(in_flight, &stab_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}








