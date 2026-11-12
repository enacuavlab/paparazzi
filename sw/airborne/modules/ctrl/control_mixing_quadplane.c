/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr> 
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
 * @author Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr>
 * @brief Manual and autonomous quadplane control mixing.
 * Control mixing for quadplane
 * 
 * Manual control:
 * - ATT: RC attitude and vertical throttle control using the hovering motors;
 *   the pusher is stopped and the elevons are neutral.
 * - FWD: transition to RC roll/pitch and pusher throttle control.
 *   The pusher ramps up, elevons become active at the transition cutoff,
 *   and vertical thrust decreases until the hovering motors are commanded off.
 *   Airspeed gates transition progress; a timeout restores quad control.
 *
 * NAV configuration selection through cmq_nav_mode:
 * - CMQ_NAV_AUTO: request forward flight for ROUTE and CIRCLE, quad otherwise.
 * - CMQ_NAV_ATTITUDE: use quad configuration for any navigation trajectory.
 * - CMQ_NAV_FORWARD: explicitly request a forward transition and configuration.
 * Navigation guidance follows the flight plan in both configurations.
 * AUTO is the startup default. Explicit selections persist across blocks
 * until cmq_nav_mode is assigned again. HOME retains quad configuration.
 *
 * Independent flight-plan block examples:
 * @code{.xml}
 * <block name="Quad route">
 *   <set var="cmq_nav_mode" value="CMQ_NAV_ATTITUDE"/>
 *   <go wp="p1" hmode="route"/>
 * </block>
 *
 * <block name="Forward route">
 *   <set var="cmq_nav_mode" value="CMQ_NAV_FORWARD"/>
 *   <go wp="p2" from="p1" hmode="route"/>
 * </block>
 *
 * <block name="Quad hold">
 *   <set var="cmq_nav_mode" value="CMQ_NAV_ATTITUDE"/>
 *   <stay wp="p2"/>
 * </block>
 *
 * <block name="forward_hold">
 *   <set var="cmq_nav_mode" value="CMQ_NAV_FORWARD"/>
 *   <circle wp="p2" radius="nav.radius"/>
 * </block>
 *
 * <block name="Auto route">
 *   <set var="cmq_nav_mode" value="CMQ_NAV_AUTO"/>
 *   <go wp="p2" from="p1" hmode="route"/>
 * </block>
 * @endcode
 *
 * Forward holding uses an orbit around the waypoint; quad holding uses
 * a stationary position setpoint.
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
#include <string.h>
#include "math/pprz_isa.h"

// Use for Elevon test on ground, turn off motors and allow only elevon to move.
#ifndef CMQ_GROUND_ELEVON_TEST
#define CMQ_GROUND_ELEVON_TEST FALSE
#endif

// Time for forward transition
#ifndef CMQ_TRANSITION_TIME
#define CMQ_TRANSITION_TIME 6.f
#endif

// Transition airspeed, set a negative value to disable
#ifndef CMQ_TRANSITION_AIRSPEED
#define CMQ_TRANSITION_AIRSPEED 13.f
#endif

// Transition minimum pusher command
#ifndef CMQ_TRANSITION_PUSHER_CMD
#define CMQ_TRANSITION_PUSHER_CMD 9600.f
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

#ifndef CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY
#define CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY PERIODIC_FREQUENCY
#endif

#ifndef CMQ_FORWARD_MIN_AIRSPEED
#define CMQ_FORWARD_MIN_AIRSPEED 15.f
#endif

// Rotor cut duration (CUTOFF -> 1) once the airspeed threshold is confirmed
#ifndef CMQ_TRANSITION_COMPLETION_TIME
#define CMQ_TRANSITION_COMPLETION_TIME 1.f
#endif

// Airspeed must stay above the threshold during this time (pitot spikes)
#ifndef CMQ_TRANSITION_AIRSPEED_CONFIRM_TIME
#define CMQ_TRANSITION_AIRSPEED_CONFIRM_TIME 0.2f
#endif

#define TRANSITION_RATIO_COMPLETE 0.99f


static float transition_ratio; // 0. for hover to 1. for forward flight
static const float transition_increment = 1.f / (CMQ_TRANSITION_TIME * CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY); // Increment per periodic cycle
static const float transition_completion_increment = (1.f - CMQ_TRANSITION_CUTOFF) / (CMQ_TRANSITION_COMPLETION_TIME * CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY);
static uint32_t transition_airspeed_cycles = 0;
static bool transition_airspeed_reached = false; // Latched: the transition must complete
float   quadplane_heading_sp = 0.f;           // heading setpoint [rad, NED]
static uint32_t manual_transition_cycles = 0; // Count cycles of a forward transition in manual mode, for timeout detection.
static bool manual_transition_aborted = false; // Latch to prevent repeated transition attempts when the pilot requests forward flight but the transition fails.
enum CmqNavMode cmq_nav_mode = CMQ_NAV_AUTO; //Default in AUTO, so attitude or forward depends on NAV_HORIZONTAL_MODE_ROUTE or NAV_HORIZONTAL_MODE_CIRCLE if no flightplan definition.  
static uint32_t nav_transition_cycles = 0; // Count cycles of a forward transition in NAV mode, for timeout detection.
static bool nav_transition_aborted = false; // Latch to prevent repeated transition attempts when the flight plan requests forward flight but the transition fails.
static bool nav_previous_forward_request = false; //Use to detect a new forward request and capture the heading setpoint at the beginning of the transition.

/**
 * Reurn true if the flight plan requests a quadplane forward transition
 * Or if the flight plan is in AUTO and the navigation mode is ROUTE or CIRCLE
 */
static bool quadplane_nav_forward_requested(void)
{
  /* HOME retains quad navigation. */
  if (autopilot_get_mode() != AP_MODE_NAV) {
    return false;
  }

  switch (cmq_nav_mode) {
    case CMQ_NAV_FORWARD:
      return true;

    case CMQ_NAV_AUTO:
      return nav.horizontal_mode == NAV_HORIZONTAL_MODE_ROUTE ||
             nav.horizontal_mode == NAV_HORIZONTAL_MODE_CIRCLE;

    case CMQ_NAV_ATTITUDE:
    default:
      return false;
  }
}

/**
 * Return telemetry status of the quadplane transition state
 */
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

enum CmqTransitionState {
  CMQ_HOVER = 0,
  CMQ_TRANSITION = 1,
  CMQ_FORWARD = 2,
  CMQ_ABORTED = 3,
  CMQ_WAITING = 4
};

static void send_transition_cmq_status(struct transport_tx *trans,
                                       struct link_device *dev)
{
  static char state_names[][11] = {
    [CMQ_HOVER]      = "HOVER",
    [CMQ_TRANSITION] = "TRANSITION",
    [CMQ_FORWARD]    = "FORWARD",
    [CMQ_ABORTED]    = "ABORTED",
    [CMQ_WAITING]    = "WAITING"
  };

  uint8_t status = CMQ_HOVER;
  float ratio = 0.f;

  const uint8_t mode = autopilot_get_mode();

  const bool manual_forward = mode == AP_MODE_FORWARD;
  const bool automatic_forward =
    quadplane_nav_forward_requested();

  if (manual_forward || automatic_forward) {
    ratio = transition_ratio;

    const bool aborted = manual_forward
        ? manual_transition_aborted
        : nav_transition_aborted;

    if (aborted) {
      status = CMQ_ABORTED;
    } else if (!autopilot_in_flight() || !autopilot_get_motors_on()) {
      status = CMQ_WAITING;
    } else if (transition_ratio >= 1.f && quadplane_eff_forward_mode) {
      status = CMQ_FORWARD;
    } else {
      status = CMQ_TRANSITION;
    }
  }

  char *state_text = state_names[status];

  pprz_msg_send_TRANSITION_CMQ_STATUS(
      trans, dev, AC_ID,
      &ratio,
      (uint8_t)strlen(state_text), state_text);
}
#endif

/**
 * Forward / Hover transition functions
 */
static void transition_run(bool to_forward) {
  if (to_forward && transition_ratio < 1.f) {
    const bool airspeed_valid = stateIsAirspeedValid();
    const float airspeed = stateGetAirspeed_f();
    const uint32_t confirm_cycles = (uint32_t)ceilf(
        CMQ_TRANSITION_AIRSPEED_CONFIRM_TIME *
        (float)CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY);

    // A negative threshold disables the airspeed gate: no early completion
    if (airspeed_valid && CMQ_TRANSITION_AIRSPEED > 0.f &&
        airspeed >= CMQ_TRANSITION_AIRSPEED) {
      transition_airspeed_cycles++;
    } else {
      transition_airspeed_cycles = 0;
    }
    if (transition_airspeed_cycles >= confirm_cycles) {
      transition_airspeed_reached = true;
    }

    if (transition_airspeed_reached) {
      transition_ratio += Max(transition_increment, transition_completion_increment);
    } else if (airspeed_valid &&
               (airspeed >= CMQ_TRANSITION_AIRSPEED ||
                transition_ratio + transition_increment < CMQ_TRANSITION_CUTOFF)) {
      transition_ratio += transition_increment;
    }

    if (transition_ratio >= TRANSITION_RATIO_COMPLETE) {
      transition_ratio = 1.f;
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

static void quadplane_force_hover(void)
{
  transition_ratio = 0.f; // 0. for hover to 1. for forward flight
  transition_airspeed_cycles = 0;
  transition_airspeed_reached = false;

  quadplane_eff_disable_elevons = true;
  quadplane_eff_disable_pusher = true;
  quadplane_eff_pusher_forced = false;
  quadplane_eff_pusher_forced_cmd = 0.f;
  quadplane_eff_forward_mode = false;
  quadplane_eff_hover_thrust_factor = 1.f;
}

void control_mixing_quadplane_init(void)
{
  quadplane_force_hover();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic,
                             PPRZ_MSG_ID_TRANSITION_CMQ_STATUS,
                             send_transition_cmq_status);
#endif  
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
  quadplane_eff_disable_elevons = false;
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

  stabilization_run(true, &stab_sp, &thrust_sp, stabilization.cmd);

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
  quadplane_force_hover();
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_quadplane_attitude_direct(void)
{
  const bool in_flight = autopilot_in_flight();
  quadplane_force_hover();
  transition_run(TRANSITION_TO_HOVER);
  struct ThrustSetpoint th_sp = guidance_v_run(in_flight);
  stabilization_run(in_flight, &stabilization.rc_sp, &th_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, in_flight, autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}


/* Forward : Forward mode in manual for quadplane */
void control_mixing_quadplane_attitude_plane_enter(void)
{
  // don't use forward submode to avoid pitch offset on RC input
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  manual_transition_cycles = 0;
  manual_transition_aborted = false;
}

void control_mixing_quadplane_attitude_plane(void)
{
  const bool in_flight = autopilot_in_flight();
  const bool motors_on = autopilot_get_motors_on();
  int32_t thrust[3] = { 0, 0, 0 };
  struct StabilizationSetpoint attitude_sp;

  const uint32_t timeout_cycles = (uint32_t)ceilf(CMQ_TRANSITION_TIME * (float)CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY);

  // *** Check on timeout Part *** //
  //On ground
  if (!in_flight || !motors_on) {
    manual_transition_cycles = 0;
    control_mixing_quadplane_attitude_direct();
    return;
  }

  //After Timeout, abort the transition and go back to attitude direct mode
  if (manual_transition_aborted) {
    control_mixing_quadplane_attitude_direct();
    return;
  }

  quadplane_pusher_gate(in_flight && motors_on);
  transition_run(TRANSITION_TO_FORWARD);

  //Check if the transition is completed, if not, count the cycles and abort if timeout reached
  if (transition_ratio >= 1.f || transition_airspeed_reached) {
    manual_transition_cycles = 0;
  } else {
    manual_transition_cycles++;

    if (manual_transition_cycles >= timeout_cycles) {
      manual_transition_aborted = true;
      //Go back in attitude mode, the transition is aborted
      control_mixing_quadplane_attitude_direct_enter();
      control_mixing_quadplane_attitude_direct();
      return;
    }
  }

  // *** Transition Part *** //
  const bool plane_mode = transition_ratio >= CMQ_TRANSITION_CUTOFF;
  const float hov_motor_decrease = transition_hov_motor(transition_ratio, CMQ_TRANSITION_CUTOFF, 1.f);
  struct ThrustSetpoint z_sp = guidance_v_run(in_flight);
  const int32_t hover_tz = th_sp_to_thrust_i(&z_sp, 0, THRUST_AXIS_Z);
  float push_ratio = transition_ratio / CMQ_TRANSITION_CUTOFF;
  Bound(push_ratio, 0.f, 1.f);



  //Case 1 : Forward requested and not yet in plane mode, increase pusher command progressively until cutoff
  //MODE QUAD => RC rotorcraft : roll + pitch + yaw + thrust Z.
  if (!plane_mode) {
    attitude_sp = stabilization.rc_sp;

    thrust[THRUST_AXIS_Z] = hover_tz;
    thrust[THRUST_AXIS_X] = push_ratio * CMQ_TRANSITION_PUSHER_CMD;
    //commands[COMMAND_THRUST] = hover_tz;
    //commands[COMMAND_THRUST_X] = push_ratio * CMQ_TRANSITION_PUSHER_CMD;
    
    quadplane_eff_pusher_forced = true;
    quadplane_eff_pusher_forced_cmd = push_ratio * CMQ_TRANSITION_PUSHER_CMD;
    quadplane_eff_forward_mode = false;
    //Keep elevons disabled until we reach the cutoff, to avoid any control conflict between elevons and quad motors
    quadplane_eff_disable_elevons = true;
    //Display in GCS the 4 motors mean thrust
    autopilot.throttle = commands[COMMAND_THRUST];

  //Case 2 : Forward requested and in plane mode, unleash the pusher command and let the INDI/WLS control it
  //MODE PLANE => RC plane : roll + pitch + thrust X.
  } else {
    struct FloatEulers plane_rc_sp = {
      .phi = STABILIZATION_ATTITUDE_SP_MAX_PHI * radio_control_get(RADIO_ROLL) / MAX_PPRZ,
      .theta = STABILIZATION_ATTITUDE_SP_MAX_THETA * radio_control_get(RADIO_PITCH) / MAX_PPRZ,
      .psi = stateGetNedToBodyEulers_f()->psi
    };
    attitude_sp = stab_sp_from_eulers_f(&plane_rc_sp);

    //Unlock the elevons
    quadplane_eff_disable_elevons = false;
    //X Axis
    thrust[THRUST_AXIS_X] = Max(radio_control_get(RADIO_THROTTLE), 0);
    //commands[COMMAND_THRUST_X] = Max(radio_control_get(RADIO_THROTTLE), 0);
    quadplane_eff_pusher_forced = false;
    quadplane_eff_pusher_forced_cmd = 0.f;

    //Z Axis
    if (transition_ratio < 1.f) {
      //Progressive reduction of vertical thrust
      thrust[THRUST_AXIS_Z] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
      //commands[COMMAND_THRUST] = (int32_t)((1.f - hov_motor_decrease) * hover_tz);
      quadplane_eff_forward_mode = false;
    } else {
      //Transition completed, stop vertical thrust and let the INDI/WLS control the pusher
      thrust[THRUST_AXIS_Z] = 0;
      //commands[COMMAND_THRUST] = 0;
      quadplane_eff_forward_mode = true;
    }
    //Display in GCS the pusher motors thrust
    autopilot.throttle = Max(commands[COMMAND_THRUST_X], 0);
  }

  struct ThrustSetpoint thrust_sp = th_sp_from_thrust_vect_i(thrust);
  stabilization_run(in_flight, &attitude_sp, &thrust_sp, stabilization.cmd);
  SetRotorcraftCommands(stabilization.cmd, in_flight, motors_on);
}

void control_mixing_quadplane_guidance_enter(void)
{
//TODO
}

void control_mixing_quadplane_guidance(void)
{
//TODO
}

void control_mixing_quadplane_nav_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
  guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  nav_transition_cycles = 0;
  nav_transition_aborted = false;
  nav_previous_forward_request = false;
}

void control_mixing_quadplane_nav(void)
{
  const bool in_flight = autopilot_in_flight();
  const bool motors_on = autopilot_get_motors_on();

  //The flight plan selects the aircraft configuration.
  //HOME retains quad navigation independently of this NAV selector.
  const bool forward_request =
      quadplane_nav_forward_requested() &&
      in_flight &&
      motors_on;

  const uint32_t timeout_cycles = (uint32_t)ceilf(
      CMQ_TRANSITION_TIME *
      (float)CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY);

  /* Capture the requested flight direction when starting a transition. */
  if (forward_request && !nav_previous_forward_request) {
    const struct EnuCoor_f *position = stateGetPositionEnu_f();

    float east = nav.carrot.x - position->x;
    float north = nav.carrot.y - position->y;

    if (nav.setpoint_mode == NAV_SETPOINT_MODE_SPEED) {
      east = nav.speed.x;
      north = nav.speed.y;
    }

    quadplane_heading_sp =
        east * east + north * north > 0.01f
        ? atan2f(east, north)
        : stateGetNedToBodyEulers_f()->psi;
  }

  /* Capture the current heading when returning to quad navigation. */
  if (!forward_request && nav_previous_forward_request) {
    nav.heading = stateGetNedToBodyEulers_f()->psi;
    guidance_h_set_heading(nav.heading);
  }

  /*
   * An abort remains latched while FORWARD is requested.
   * Selecting ATTITUDE, or entering NAV again, permits another attempt.
   */
  if (!forward_request) {
    nav_transition_cycles = 0;
    nav_transition_aborted = false;
    quadplane_force_hover();
    //We reset the YOLO heading for quad navigation
    nav.heading = stateGetNedToBodyEulers_f()->psi;
    guidance_h_set_heading(nav.heading);

  } else if (nav_transition_aborted) {
    quadplane_force_hover();

  } else {
    /*
     * transition_run() checks airspeed validity and the speed threshold.
     * A blocked transition therefore eventually reaches the timeout.
     */
    transition_run(TRANSITION_TO_FORWARD);

    if (transition_ratio >= 1.f || transition_airspeed_reached) {
      nav_transition_cycles = 0;
    } else {
      nav_transition_cycles++;

      if (nav_transition_cycles >= timeout_cycles) {
        nav_transition_aborted = true;
        quadplane_force_hover();
        nav.heading = stateGetNedToBodyEulers_f()->psi;
        guidance_h_set_heading(nav.heading);
      }
    }
  }

  nav_previous_forward_request = forward_request;

  const bool transition_active =
      forward_request && !nav_transition_aborted;

  const bool plane_mode =
      transition_active &&
      transition_ratio >= CMQ_TRANSITION_CUTOFF;

  //Activation in plane mode of the heading following for hybrid quadplanes with no yaw control (e.g. Flying Wing or Tailsitter)    
  take_heading_control = !plane_mode;
  guidance_indi_heading_follow = plane_mode;

  // Set the low airspeed limits for nav in forward mode.
  guidance_set_min_max_airspeed(
      plane_mode ? CMQ_FORWARD_MIN_AIRSPEED : -10.f,
      gih_params.max_airspeed);


  /*
   * Explicitly update this flag on every cycle.
   * The scheduler stops the hovering motors only after completion.
   */
  quadplane_eff_forward_mode =
      transition_active && transition_ratio >= 1.f;

  float hov_motor_decrease = 0.f;
  if (plane_mode) {
    hov_motor_decrease = transition_hov_motor(
        transition_ratio, CMQ_TRANSITION_CUTOFF, 1.f);
  }
  quadplane_eff_hover_thrust_factor = 1.f - hov_motor_decrease;

  float push_ratio = transition_ratio / CMQ_TRANSITION_CUTOFF;
  Bound(push_ratio, 0.f, 1.f);

  /*
   * Set actuator availability before guidance and stabilization.
   * quadplane_force_hover() already handles the inactive cases.
   */
  if (transition_active) {
    quadplane_pusher_gate(in_flight && motors_on);

    quadplane_eff_disable_elevons = !plane_mode;
    quadplane_eff_pusher_forced = !plane_mode;
    quadplane_eff_pusher_forced_cmd =
        plane_mode ? 0.f : push_ratio * CMQ_TRANSITION_PUSHER_CMD;
  }

  /*
   * Use the forward speed limit in forward configuration.
   * guidance_h_from_nav() handles explicit flight-plan speed limits.
   */
  if (nav.fp_max_speed <= 0.f) {
    if (transition_active) {
      guidance_h_SetMaxSpeed(gih_params.max_airspeed);
    } else {
      guidance_h_SetMaxSpeed(GUIDANCE_H_REF_MAX_SPEED);
    }
  }

  /* Prepare vertical references, then run hybrid guidance. */
  struct ThrustSetpoint nav_thrust = guidance_v_run(in_flight);
  struct StabilizationSetpoint attitude_sp =
      guidance_h_from_nav(in_flight);

  /*
   * With the current hybrid INDI implementation, these callbacks return
   * the shared thrust setpoint. Read it after horizontal guidance has
   * computed the current cycle.
   */
  if (nav.horizontal_mode != NAV_HORIZONTAL_MODE_NONE &&
      nav.horizontal_mode != NAV_HORIZONTAL_MODE_ATTITUDE) {
    if (nav.vertical_mode == NAV_VERTICAL_MODE_ALT) {
      nav_thrust = guidance_v_run_pos(in_flight, &guidance_v);
    } else if (nav.vertical_mode == NAV_VERTICAL_MODE_CLIMB) {
      nav_thrust = guidance_v_run_speed(in_flight, &guidance_v);
    }
  }

  /*
   * Convert hybrid INDI acceleration increments into absolute
   * PPRZ commands using the stabilization effectiveness model.
   */
  float hover_tz = Max(stabilization.cmd[COMMAND_THRUST], 0);
  float pusher_tx = Max(stabilization.cmd[COMMAND_THRUST_X], 0);

  if (nav_thrust.type == THRUST_INCR_SP) {
    float collective_eff = 0.f;
    float filtered_z = 0.f;

    for (uint8_t i = 0; i < 4; i++) {
      const float eff = g1g2[QUADPLANE_VC_AZ][i];

      collective_eff += eff;
      filtered_z += eff * actuator_state_filt_vect[i];
    }

    if (collective_eff < -1.e-6f) {
      const float increment_z =
          th_sp_to_incr_f(&nav_thrust, 0, THRUST_AXIS_Z);

      hover_tz = (filtered_z + increment_z) / collective_eff;
    }

    const float pusher_eff =
        g1g2[QUADPLANE_VC_AX][QUADPLANE_ACT_PUSHER];

    if (pusher_eff > 1.e-6f) {
      const float increment_x =
          th_sp_to_incr_f(&nav_thrust, 0, THRUST_AXIS_X);

      pusher_tx =
          actuator_state_filt_vect[QUADPLANE_ACT_PUSHER] +
          increment_x / pusher_eff;
    }

  } else {
    /* Direct thrust commands, for example during engine-start stages. */
    hover_tz = th_sp_to_thrust_i(&nav_thrust, 0, THRUST_AXIS_Z);
    pusher_tx = th_sp_to_thrust_i(&nav_thrust, 0, THRUST_AXIS_X);
  }

  Bound(hover_tz, 0.f, (float)MAX_PPRZ);
  Bound(pusher_tx, 0.f, (float)MAX_PPRZ);

  int32_t thrust[3] = { 0, 0, 0 };

  if (!transition_active) {
    /* Quad navigation, including transition abort. */
    thrust[THRUST_AXIS_Z] = (int32_t)hover_tz;

  } else if (!plane_mode) {
    /* Transition before cutoff: fixed attitude and pusher ramp. */
    struct FloatEulers transition_sp = {
      .phi = 0.f,
      .theta = CMQ_TRANSITION_PITCH,
      .psi = quadplane_heading_sp
    };

    attitude_sp = stab_sp_from_eulers_f(&transition_sp);

    thrust[THRUST_AXIS_Z] = (int32_t)hover_tz;
    thrust[THRUST_AXIS_X] =
        (int32_t)quadplane_eff_pusher_forced_cmd;

  } else {
    /* After cutoff: navigation controls attitude and forward thrust. */
    thrust[THRUST_AXIS_X] = (int32_t)pusher_tx;

    if (quadplane_eff_forward_mode) {
      /* Transition complete: no vertical motor thrust requested. */
      thrust[THRUST_AXIS_Z] = 0;
    } else {
      thrust[THRUST_AXIS_Z] = (int32_t)(quadplane_eff_hover_thrust_factor * hover_tz);
    }
  }

  struct ThrustSetpoint thrust_sp = th_sp_from_thrust_vect_i(thrust);

  stabilization_run(
      in_flight, &attitude_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, in_flight, motors_on);

  const uint8_t displayed_command =
      plane_mode ? COMMAND_THRUST_X : COMMAND_THRUST;

  autopilot.throttle =
      motors_on ? Max(commands[displayed_command], 0) : 0;
}

void control_mixing_quadplane_failsafe(void)
{
  bool in_flight = autopilot_in_flight();
  quadplane_pusher_gate(in_flight && autopilot_get_motors_on());

  struct StabilizationSetpoint stab_sp = stabilization_get_failsafe_sp();
  struct ThrustSetpoint thrust_sp = guidance_v_run(in_flight);
  stabilization_run(in_flight, &stab_sp, &thrust_sp, stabilization.cmd);

  SetRotorcraftCommands(stabilization.cmd, in_flight, autopilot_get_motors_on());
  autopilot.throttle = commands[COMMAND_THRUST];
}

// Assumes LIFT_CURVE_SLOPE is the finite-wing lift-curve slope
// and airspeed is equivalent airspeed (EAS).
float guidance_indi_get_liftd(float airspeed, float theta)
{
  (void)theta;

  const float lift_accel_asq =
      0.5f * PPRZ_ISA_AIR_DENSITY * quadplane_eff_sched_p.wing_area
      * quadplane_eff_sched_p.lift_curve_slope / quadplane_eff_sched_p.mass;

  return -lift_accel_asq * airspeed * airspeed;
}