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

// Use for Elevon test on ground, turn off motors and allow only elevon to move.
#ifndef CMQ_GROUND_ELEVON_TEST
#define CMQ_GROUND_ELEVON_TEST FALSE
#endif

// Time for forward transition
#ifndef CMQ_TRANSITION_TIME
#define CMQ_TRANSITION_TIME 4.f
#endif

// Timeout for forward transition
#ifndef CMQ_TRANSITION_TIMEOUT
#define CMQ_TRANSITION_TIMEOUT 10.f
#endif

// Transition airspeed, set a negative value to disable
#ifndef CMQ_TRANSITION_AIRSPEED
#define CMQ_TRANSITION_AIRSPEED 10.f
#endif

// Transition minimum pusher command
#ifndef CMQ_TRANSITION_PUSHER_CMD
#define CMQ_TRANSITION_PUSHER_CMD 7000.f
#endif

// Transition ratio cutoff (reduce quad motors)
#ifndef CMQ_TRANSITION_CUTOFF
#define CMQ_TRANSITION_CUTOFF 0.8f
#endif

#define TRANSITION_TO_HOVER false
#define TRANSITION_TO_FORWARD true

#ifndef CMQ_TRANSITION_PITCH
#define CMQ_TRANSITION_PITCH 0.f  // radians
#endif

static float transition_ratio;
static const float transition_increment = 1.f / (CMQ_TRANSITION_TIME * PERIODIC_FREQUENCY);
float   quadplane_heading_sp = 0.f;           // heading setpoint [rad, NED]

/**
 * Forward / Hover transition functions
 */
static void transition_run(bool to_forward) {
  if (to_forward && transition_ratio < 1.f) {
    if (stateIsAirspeedValid()) {
      if ((stateGetAirspeed_f() < CMQ_TRANSITION_AIRSPEED && transition_ratio + transition_increment < CMQ_TRANSITION_CUTOFF)
          || stateGetAirspeed_f() >= CMQ_TRANSITION_AIRSPEED) {
        transition_ratio += transition_increment;
      }
    }
  } else if (!to_forward && transition_ratio > 0.f) {
    transition_ratio -= transition_increment;
  }
  Bound(transition_ratio, 0.f, 1.f);
}

static float transition_hov_motor(float ratio, float start, float end)
{
  float hov_motor_decrease = (ratio - start) / (end - start);
  Bound(hov_motor_decrease, 0.f, 1.f);
  return hov_motor_decrease;
}

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
  transition_ratio = 0.f; // 0. for hover to 1. for forward flight
}

void control_mixing_quadplane_elevon_test_enter(void)
{
  autopilot_set_in_flight(false);
  autopilot_set_motors_on(false);

  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);

  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);

  transition_ratio = 0.f;

  quadplane_eff_ground_test = true;
  quadplane_eff_forward_mode = true;
  quadplane_eff_disable_pusher = true;
  quadplane_eff_pusher_forced = false;
}

void control_mixing_quadplane_elevon_test(void)
{
  int32_t thrust[3] = { 0, 0, 0 };

  const struct FloatEulers *att = stateGetNedToBodyEulers_f();

  struct FloatEulers rc_sp = {
    .phi = STABILIZATION_ATTITUDE_SP_MAX_PHI * radio_control_get(RADIO_ROLL) / MAX_PPRZ,
    .theta = STABILIZATION_ATTITUDE_SP_MAX_THETA * radio_control_get(RADIO_PITCH) / MAX_PPRZ,
    .psi = att->psi
  };

  struct StabilizationSetpoint stab_sp = stab_sp_from_eulers_f(&rc_sp);
  struct ThrustSetpoint thrust_sp = th_sp_from_thrust_vect_i(thrust);

  stabilization_run(false, &stab_sp, &thrust_sp, stabilization.cmd);

  // Elevon output calculated by INDI/WLS.
  commands[COMMAND_ELEVON_RIGHT] = stabilization.cmd[COMMAND_ELEVON_RIGHT];
  commands[COMMAND_ELEVON_LEFT] = stabilization.cmd[COMMAND_ELEVON_LEFT];

  // All motors off.
  commands[COMMAND_MOTOR_FRONT_RIGHT] = MIN_PPRZ;
  commands[COMMAND_MOTOR_BACK_RIGHT]  = MIN_PPRZ;
  commands[COMMAND_MOTOR_BACK_LEFT]   = MIN_PPRZ;
  commands[COMMAND_MOTOR_FRONT_LEFT]  = MIN_PPRZ;
  commands[COMMAND_THRUST_X]          = MIN_PPRZ;

  autopilot.throttle = 0;
}

/* Attitude direct : Quad only mode in manual for quadplane */
void control_mixing_quadplane_attitude_direct_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_quadplane_attitude_direct(void)
{
  quadplane_eff_forward_mode = false;
  quadplane_eff_pusher_forced = false;
  transition_run(TRANSITION_TO_HOVER);
  commands[COMMAND_YAW] = 0;
  commands[COMMAND_MOTOR_PUSHER] = MIN_PPRZ;
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}


/* Forward : Forward mode in manual for quadplane */
void control_mixing_quadplane_attitude_plane_enter(void)
{
  // don't use forward submode to avoid pitch offset on RC input
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_quadplane_attitude_plane(void)
{

  const bool in_flight = autopilot_in_flight();
  const bool plane_mode = (transition_ratio >= CMQ_TRANSITION_CUTOFF);
  int32_t thrust[3] = { 0, 0, 0 };
  struct StabilizationSetpoint attitude_sp;

  //Activate pusher if in flight
  quadplane_pusher_gate(in_flight);
  transition_run(TRANSITION_TO_FORWARD);
  float hov_motor_decrease = transition_hov_motor(transition_ratio, CMQ_TRANSITION_CUTOFF, 1.f);

  struct ThrustSetpoint z_sp = guidance_v_run(in_flight);
  const int32_t hover_tz = th_sp_to_thrust_i(&z_sp, 0, THRUST_AXIS_Z);
    /*
     * MODE QUAD => RC rotorcraft : roll + pitch + yaw + thrust Z.
     */
  if (!plane_mode) {
    attitude_sp = stabilization.rc_sp;

    //commands[COMMAND_THRUST] = hover_tz;
    //commands[COMMAND_THRUST_X] = CMQ_TRANSITION_PUSHER_CMD;
    thrust[THRUST_AXIS_Z] = hover_tz;
    thrust[THRUST_AXIS_X] = CMQ_TRANSITION_PUSHER_CMD;
    quadplane_eff_pusher_forced_cmd = CMQ_TRANSITION_PUSHER_CMD;
    quadplane_eff_pusher_forced = true;
    quadplane_eff_forward_mode = false;
  } else {
    /*
     * MODE PLANE => RC plane : roll + pitch + thrust X.
     */
    struct FloatEulers plane_rc_sp = {
      .phi = STABILIZATION_ATTITUDE_SP_MAX_PHI * radio_control_get(RADIO_ROLL) / MAX_PPRZ,
      .theta = STABILIZATION_ATTITUDE_SP_MAX_THETA * radio_control_get(RADIO_PITCH) / MAX_PPRZ,
      .psi = 0.f
    };
    attitude_sp = stab_sp_from_eulers_f(&plane_rc_sp);

    //X Axis
    //commands[COMMAND_THRUST_X] = Max(radio_control_get(RADIO_THROTTLE), 0);
    //commands[COMMAND_THRUST_X] = radio_control_get(RADIO_THROTTLE);
    thrust[THRUST_AXIS_X] =Max(radio_control_get(RADIO_THROTTLE), 0);
    quadplane_eff_pusher_forced = false;
    //Z Axis
    if(transition_ratio < 1.f) {
      //commands[COMMAND_THRUST] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
      thrust[THRUST_AXIS_Z] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
      quadplane_eff_forward_mode = false;
    } else if(transition_ratio >= 1.f) {
      //commands[COMMAND_THRUST] = 0; //We stop the vertical thrust when in forward mode
      thrust[THRUST_AXIS_Z] = 0; //We stop the vertical thrust when in forward mode
      quadplane_eff_forward_mode = true;
    }
  }

  struct ThrustSetpoint thrust_sp = th_sp_from_thrust_vect_i(thrust);

  stabilization_run(in_flight, &attitude_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, in_flight, autopilot_get_motors_on());
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



void control_mixing_quadplane_nav(void)
{
  static bool previous_forward_request = false;
  static bool transition_heading_initialized = false; // Flag to initialize the heading setpoint at the beginning of the transition


  //Forward / Hover transition
  const bool in_flight = autopilot_in_flight();
  const bool motors_on = autopilot_get_motors_on();
  const bool forward_nav = nav.horizontal_mode == NAV_HORIZONTAL_MODE_ROUTE || nav.horizontal_mode == NAV_HORIZONTAL_MODE_CIRCLE;
  const bool forward_request = forward_nav && in_flight && motors_on && stateIsAirspeedValid();
  float hov_motor_decrease = 0.f;

  quadplane_pusher_gate(in_flight && motors_on);


  //Capture heading when returning to hover, do not overwrite nav.heading at each forward cycle
  if (previous_forward_request && !forward_request) {
    nav.heading = stateGetNedToBodyEulers_f()->psi;
    guidance_h_set_heading(nav.heading);
  }

  struct ThrustSetpoint th_sp = guidance_v_run(in_flight);
  const int32_t hover_tz = th_sp_to_thrust_i(&th_sp, 0, THRUST_AXIS_Z);
  struct StabilizationSetpoint attitude_sp;


  //If motors ON and in flight, run transition and save the forward request
  if (!in_flight || !motors_on) {
    transition_ratio = 0.f;
    quadplane_eff_disable_pusher = true;
  } else {
    transition_run(forward_request ? TRANSITION_TO_FORWARD : TRANSITION_TO_HOVER);
    hov_motor_decrease = transition_hov_motor(transition_ratio, CMQ_TRANSITION_CUTOFF, 1.f);
  }

  previous_forward_request = forward_request;

  // Plane mode calculation after transition_run() to avoid using the previous cycle ratio
  const bool plane_mode = forward_request && transition_ratio >= CMQ_TRANSITION_CUTOFF;

  //Pusher transition command definition at each step
  quadplane_eff_pusher_forced = false;
  quadplane_eff_pusher_forced_cmd = 0.f;

  //Transition to forward flight, increase pusher command until cutoff, then maintain the command until forward is established
  float push_ratio = transition_ratio / CMQ_TRANSITION_CUTOFF;
  Bound(push_ratio, 0.f, 1.f);

  //Case 1 : Forward requested and not yet in plane mode, increase pusher command progressively until cutoff
  if (forward_request && !plane_mode) {
    //Progressive increase of the pusher during the first phase. At cutoff, it reaches CMQ_TRANSITION_PUSHER_CMD, when push_ratio reaches 1
    //Set heading setpoint to the current heading at the beginning of the transition, and keep it until the end of the transition
    if (!transition_heading_initialized) {
      const struct EnuCoor_f *start = stateGetPositionEnu_f();
      const float east  = nav.carrot.x - start->x;
      const float north = nav.carrot.y - start->y;

      quadplane_heading_sp = start->x == nav.carrot.x && start->y == nav.carrot.y ? stateGetNedToBodyEulers_f()->psi : atan2f(east, north);

      transition_heading_initialized = true;
    }
    //Set the heading setpoint to the captured heading during the transition and the pitch for transition
    struct FloatEulers eulers_sp = { .phi = 0.f , .theta = CMQ_TRANSITION_PITCH, .psi = quadplane_heading_sp };
    attitude_sp = stab_sp_from_eulers_f(&eulers_sp);    
    quadplane_eff_pusher_forced = true;
    quadplane_eff_pusher_forced_cmd = push_ratio * CMQ_TRANSITION_PUSHER_CMD;
    quadplane_eff_forward_mode = false;

  //Case 2 : Returning to hover, decrease pusher command proportionally to the transition ratio until it reaches 0 at the end of the transition
  } else if (!forward_request) {
    //Back to Hover : pusher command is forced to the previous value, and then reduced proportionally to the transition ratio.
    attitude_sp = guidance_h_from_nav(in_flight);
    quadplane_eff_forward_mode = false;
    transition_heading_initialized = false;
    if (transition_ratio > 0.f) {
      quadplane_eff_pusher_forced = true;
      quadplane_eff_pusher_forced_cmd = push_ratio * CMQ_TRANSITION_PUSHER_CMD;
    } else if (transition_ratio <= 0.f) {
      quadplane_eff_pusher_forced = false;
      quadplane_eff_pusher_forced_cmd = 0;
    }
    
  //Case 3 : Forward requested and in plane mode, unleash the pusher command and let the INDI/WLS control it
  }  else if (forward_request && plane_mode) {
    attitude_sp = guidance_h_from_nav(in_flight);
    transition_heading_initialized = false;
    quadplane_eff_pusher_forced = false;
    quadplane_eff_pusher_forced_cmd = 0.f;

    if(transition_ratio < 1.f) {

    commands[COMMAND_THRUST] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
    //thrust[THRUST_AXIS_Z] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
    quadplane_eff_forward_mode = false;
    } else if(transition_ratio >= 1.f) {
      commands[COMMAND_THRUST] = 0; //We stop the vertical thrust when in forward mode
      //thrust[THRUST_AXIS_Z] = 0; //We stop the vertical thrust when in forward mode
      quadplane_eff_forward_mode = true;
    }
  }

  stabilization_run(in_flight, &attitude_sp, &th_sp, stabilization.cmd);
  SetRotorcraftCommands(stabilization.cmd, in_flight, motors_on);

  autopilot.throttle = commands[COMMAND_THRUST];
}




