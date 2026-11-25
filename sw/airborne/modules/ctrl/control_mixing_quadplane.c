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
 * NAV : Full INDI
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
#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
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

 // Time for forward -> hover
#ifndef CMQ_BACK_TRANSITION_TIME
#define CMQ_BACK_TRANSITION_TIME 2.f  
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

#ifndef CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY
#define CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY PERIODIC_FREQUENCY
#endif

// Rotor cut duration (CUTOFF -> 1) once the airspeed threshold is confirmed
#ifndef CMQ_TRANSITION_COMPLETION_TIME
#define CMQ_TRANSITION_COMPLETION_TIME 1.f
#endif

// Airspeed must stay above the threshold during this time (pitot spikes)
#ifndef CMQ_TRANSITION_AIRSPEED_CONFIRM_TIME
#define CMQ_TRANSITION_AIRSPEED_CONFIRM_TIME 0.2f
#endif

// quadplane_eff_rotor_authority below -> plane mode   (~12.5 m/s)
#ifndef CMQ_PLANE_MODE_A_ON
#define CMQ_PLANE_MODE_A_ON  0.1f     
#endif

// quadplane_eff_rotor_authority above -> hover mode   (~10.5 m/s)
#ifndef CMQ_PLANE_MODE_A_OFF
#define CMQ_PLANE_MODE_A_OFF 0.5f     
#endif

_Static_assert(CMQ_PLANE_MODE_A_ON < CMQ_PLANE_MODE_A_OFF, "plane mode hysteresis inverted");

// pusher allowed above this height [m]
#ifndef CMQ_PUSHER_MIN_HEIGHT
#define CMQ_PUSHER_MIN_HEIGHT 7.f      
#endif

// pusher disabled again below MIN_HEIGHT - HYST [m]
#ifndef CMQ_PUSHER_HEIGHT_HYST
#define CMQ_PUSHER_HEIGHT_HYST 4.f     
#endif

// max yaw rate of the heading command while aligning the nose before a transition [deg/s]
#ifndef CMQ_HEADING_ALIGN_RATE
#define CMQ_HEADING_ALIGN_RATE 25.f
#endif
// hover navigation: turn the nose towards the motion above this nav speed [m/s] (SPEED setpoint mode)
#ifndef CMQ_HOVER_TRACK_SPEED
#define CMQ_HOVER_TRACK_SPEED 4.f
#endif
// hover navigation: same, above this distance to the carrot [m] (POS setpoint mode, e.g. HOME)
#ifndef CMQ_HOVER_TRACK_DIST
#define CMQ_HOVER_TRACK_DIST 5.f
#endif
// wing-borne floor on route/circle [m/s]
#ifndef CMQ_FORWARD_MIN_AIRSPEED
#define CMQ_FORWARD_MIN_AIRSPEED 14.f   
#endif
// [deg] max lead of the heading command over the real heading
#ifndef CMQ_HEADING_MAX_LEAD
#define CMQ_HEADING_MAX_LEAD 30.f   
#endif
// weathervane: yaw rate per roll angle [1/s] (10 deg of roll -> 10 deg/s of yaw)
#ifndef CMQ_WVANE_GAIN
#define CMQ_WVANE_GAIN 1.f
#endif
// weathervane: active above this airspeed [m/s]
#ifndef CMQ_WVANE_MIN_AIRSPEED
#define CMQ_WVANE_MIN_AIRSPEED 3.f
#endif
// weathervane: max yaw rate [deg/s]
#ifndef CMQ_WVANE_MAX_RATE
#define CMQ_WVANE_MAX_RATE 20.f
#endif



#define TRANSITION_RATIO_COMPLETE 0.99f


static float transition_ratio; // 0. for hover to 1. for forward flight
static const float transition_increment = 1.f / (CMQ_TRANSITION_TIME * CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY); // Increment per periodic cycle
static const float transition_completion_increment = (1.f - CMQ_TRANSITION_CUTOFF) / (CMQ_TRANSITION_COMPLETION_TIME * CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY);
static const float back_transition_decrement = 1.f / (CMQ_BACK_TRANSITION_TIME * CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY); // Decrement per periodic cycle
static uint32_t transition_airspeed_cycles = 0;
static bool transition_airspeed_reached = false; // Latched: the transition must complete
static uint32_t manual_transition_cycles = 0; // Count cycles of a forward transition in manual mode, for timeout detection.
static bool manual_transition_aborted = false; // Latch to prevent repeated transition attempts when the pilot requests forward flight but the transition fails.
static bool nav_plane_mode = false;   // NAV: wing-borne state, from rotor authority only


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
  if (mode == AP_MODE_FORWARD) {
    ratio  = transition_ratio;
    status = manual_transition_aborted ? CMQ_ABORTED
           : (transition_ratio >= 1.f && quadplane_eff_forward_mode) ? CMQ_FORWARD
           : CMQ_TRANSITION;
  } else if (mode == AP_MODE_NAV || mode == AP_MODE_HOME) {
    const float a = quadplane_eff_rotor_authority;
    ratio = 1.f - a;                                   // 0 hover .. 1 wing-borne
    if (!autopilot_in_flight() || !autopilot_get_motors_on()) status = CMQ_WAITING;
    else if (nav_plane_mode)  status = CMQ_FORWARD;
    else if (a < 1.f)         status = CMQ_TRANSITION;
    else                      status = CMQ_HOVER;
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
      transition_ratio = Max(transition_ratio, CMQ_TRANSITION_CUTOFF);  //Go directly to the cutoff if the airspeed is reached
      transition_ratio += Max(transition_increment, transition_completion_increment); // cutoff -> 1. for completion of the transition
    } else if (airspeed_valid &&
                transition_ratio + transition_increment < CMQ_TRANSITION_CUTOFF) {
      transition_ratio += transition_increment; // ramp, held below cutoff until airspeed is confirmed
    }

    if (transition_ratio >= TRANSITION_RATIO_COMPLETE) {
      transition_ratio = 1.f;
    }
  } else if (!to_forward && transition_ratio > 0.f) {
    transition_airspeed_reached = false;
    transition_airspeed_cycles = 0;    
    transition_ratio -= back_transition_decrement;
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
 * The pusher is allowed only in flight and above CMQ_PUSHER_MIN_HEIGHT
 */
static inline void quadplane_pusher_gate(bool in_flight)
{
  static bool high_enough = false;
  const float height = GetPosHeight();
  if (!high_enough && height > CMQ_PUSHER_MIN_HEIGHT) {
    high_enough = true;
  } else if (high_enough && height < CMQ_PUSHER_MIN_HEIGHT - CMQ_PUSHER_HEIGHT_HYST) {
    high_enough = false;
  }
  quadplane_eff_disable_pusher = !(in_flight && high_enough);
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
  force_forward = false; //Set to false this guidance parameter, it will be set to true if the nav is in plane mode and performing a ROUTE or CIRCLE
}

void control_mixing_quadplane_nav(void)
{
  const bool in_flight = autopilot_in_flight();
  const bool motors_on = autopilot_get_motors_on();

  /* 1. Plane mode from the hover rotor authority only (airspeed driven) */
  const float a = quadplane_eff_rotor_authority;
  if (!nav_plane_mode && in_flight && motors_on && a < CMQ_PLANE_MODE_A_ON) {
    nav_plane_mode = true;
  }
  if (nav_plane_mode && (a > CMQ_PLANE_MODE_A_OFF || !in_flight)) {
    nav_plane_mode = false;
  }
  const bool plane_mode = nav_plane_mode;

  /* 2. Nose along the motion while the rotors control yaw (hover / hybrid band) */
  /*
  static float heading_cmd = 0.f;
  static bool heading_cmd_init = false;
  bool align_nose = false;
  float east = 0.f, north = 0.f;
  if (!plane_mode) {
    const struct EnuCoor_f *pos = stateGetPositionEnu_f();
    if (nav.setpoint_mode == NAV_SETPOINT_MODE_SPEED) {
      east = nav.speed.x;
      north = nav.speed.y;
      align_nose = (east * east + north * north > CMQ_HOVER_TRACK_SPEED * CMQ_HOVER_TRACK_SPEED);
    } else {
      east = nav.carrot.x - pos->x;
      north = nav.carrot.y - pos->y;
      align_nose = (east * east + north * north > CMQ_HOVER_TRACK_DIST * CMQ_HOVER_TRACK_DIST);
    }
    if (east * east + north * north < 1.e-2f) {
      align_nose = false;
    }
  }*/

  /*
  if (align_nose) {
    if (!heading_cmd_init) {
      heading_cmd = stateGetNedToBodyEulers_f()->psi;
      heading_cmd_init = true;
    }
    // Limit the rotation speed when aligning && you are not in plane mode. You limit your rotation rate to CMQ_HEADING_ALIGN_RATE
    float err = atan2f(east, north) - heading_cmd;
    FLOAT_ANGLE_NORMALIZE(err);
    const float step = RadOfDeg(CMQ_HEADING_ALIGN_RATE) / (float)CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY;
    heading_cmd += Clip(err, -step, step);
    // Limit the lead angle to avoid excessive yaw ask 
    const float psi_now = stateGetNedToBodyEulers_f()->psi;
    float lead = heading_cmd - psi_now;
    FLOAT_ANGLE_NORMALIZE(lead);
    BoundAbs(lead, RadOfDeg(CMQ_HEADING_MAX_LEAD));
    heading_cmd = psi_now + lead;
    FLOAT_ANGLE_NORMALIZE(heading_cmd);

    nav.heading = heading_cmd;
    guidance_h_set_heading(nav.heading);
  } else { 
     // Holding a point in hover (small nav speed), or plane mode (heading follows the turn) 
    heading_cmd_init = false;
    if (!plane_mode && in_flight) {
      const struct FloatEulers *e = stateGetNedToBodyEulers_f();
      const float as = stateIsAirspeedValid() ? stateGetAirspeed_f() : 0.f;
       //Weathervane: the aircraft leans into the wind -> yaw towards the lean side 
      if (as > CMQ_WVANE_MIN_AIRSPEED) {
        float rate = CMQ_WVANE_GAIN * e->phi;
        BoundAbs(rate, RadOfDeg(CMQ_WVANE_MAX_RATE));
        nav.heading += rate / (float)CONTROL_MIXING_QUADPLANE_PERIODIC_FREQUENCY;
      }
       //Anti-windup: the heading command stays within CMQ_HEADING_MAX_LEAD of psi 
      float lead = nav.heading - e->psi;
      FLOAT_ANGLE_NORMALIZE(lead);
      BoundAbs(lead, RadOfDeg(CMQ_HEADING_MAX_LEAD));
      nav.heading = e->psi + lead;
      FLOAT_ANGLE_NORMALIZE(nav.heading);
      guidance_h_set_heading(nav.heading);
    }
  }*/

  /* 3. Leaving plane mode: hold the current heading (no yaw jump) */
  /*static bool prev_plane_mode = false;
  if (prev_plane_mode && !plane_mode) {
    nav.heading = stateGetNedToBodyEulers_f()->psi;
    guidance_h_set_heading(nav.heading);
  }
  prev_plane_mode = plane_mode;*/

  /* 4. Actuator gates: ground only */
  quadplane_pusher_gate(in_flight && motors_on);
  quadplane_eff_disable_elevons = !in_flight;
  quadplane_eff_pusher_forced   = false;
  quadplane_eff_forward_mode    = false;   /* no longer used by NAV */

  /* 5. Stay in plane mode once you are in it and performing ROUTE or CIRCLE with force_forward guidance parameter */
  const bool keep_flying = plane_mode &&
      (nav.horizontal_mode == NAV_HORIZONTAL_MODE_ROUTE ||
       nav.horizontal_mode == NAV_HORIZONTAL_MODE_CIRCLE);
  force_forward = keep_flying;
  /*guidance_set_min_max_airspeed(keep_flying ? CMQ_FORWARD_MIN_AIRSPEED : -10.f,
                                gih_params.max_airspeed);
  if (nav.fp_max_speed <= 0.f) {
    guidance_h_SetMaxSpeed(GUIDANCE_H_REF_MAX_SPEED
        + (1.f - a) * (gih_params.max_airspeed - GUIDANCE_H_REF_MAX_SPEED));
  }*/


  /* 6. Heading: rotors control yaw in hover, coordinated turn in plane mode */
  //take_heading_control         = !plane_mode;
  guidance_indi_heading_follow = plane_mode;

  /* 7. Horizontal reference: off in plane mode (turns need V^2/R),
   *    on in hover, restarted from the current state on re-entry */
  static bool ref_was_off = false;
  const bool ref_off = plane_mode;

  if (ref_off) {
    guidance_h_SetUseRef(false);
    ref_was_off = true;
  } else {
    if (ref_was_off) {
      const struct NedCoor_i *pos_i = stateGetPositionNed_i();
      const struct NedCoor_f *spd_f = stateGetSpeedNed_f();
      struct Int32Vect2 ref_pos = { pos_i->x, pos_i->y };
      struct FloatVect2 ref_speed = { spd_f->x, spd_f->y };
      struct FloatVect2 ref_accel = { 0.f, 0.f };
      gh_set_ref(ref_pos, ref_speed, ref_accel);
      ref_was_off = false;
    }
    guidance_h_SetUseRef(true);
  }


  /* 8. Guidance, then direct incremental thrust to the inner INDI */
  struct ThrustSetpoint nav_thrust = guidance_v_run(in_flight);
  struct StabilizationSetpoint attitude_sp = guidance_h_from_nav(in_flight);

  /* 9. The hybrid guidance computes thrust_sp inside guidance_h_from_nav():
   *      read it again so that thrust and attitude come from the same cycle */
  if (nav.horizontal_mode != NAV_HORIZONTAL_MODE_NONE &&
      nav.horizontal_mode != NAV_HORIZONTAL_MODE_ATTITUDE) {
    if (nav.vertical_mode == NAV_VERTICAL_MODE_ALT) {
      nav_thrust = guidance_v_run_pos(in_flight, &guidance_v);
    } else if (nav.vertical_mode == NAV_VERTICAL_MODE_CLIMB) {
      nav_thrust = guidance_v_run_speed(in_flight, &guidance_v);
    }
  }

  stabilization_run(in_flight, &attitude_sp, &nav_thrust, stabilization.cmd);
  SetRotorcraftCommands(stabilization.cmd, in_flight, motors_on);
  autopilot.throttle = motors_on ? Max(commands[plane_mode ? COMMAND_THRUST_X : COMMAND_THRUST], 0) : 0;
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