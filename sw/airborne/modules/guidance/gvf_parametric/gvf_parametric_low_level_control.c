/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric/gvf_parametric_low_level_control.c
 *
 * Firmware dependent file for the guiding vector field algorithm for 2D and 3D parametric trajectories.
 */

#include "autopilot.h"
#include "gvf_parametric_low_level_control.h"
#include "gvf_parametric.h"

#if defined(FIXEDWING_FIRMWARE)
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v_n.h" // gvf_parametric is only compatible with the new pprz controller!
#include "firmwares/fixedwing/nav.h"

static float step_throttle_controller(float throttle_mod)
{
  static float throttle_delta;
  float upper_delta = v_ctl_auto_throttle_max_cruise_throttle - V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  float lower_delta = v_ctl_auto_throttle_min_cruise_throttle - V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  float range_delta = v_ctl_auto_throttle_max_cruise_throttle - v_ctl_auto_throttle_min_cruise_throttle;

  throttle_delta += ((throttle_mod > 0) - (throttle_mod < 0)) * (range_delta) / 100;
  Bound(throttle_delta,lower_delta,upper_delta);

  return throttle_delta;
}

#endif

void gvf_parametric_low_level_control_2d(float heading_rate, float throttle_mod)
{
#if defined(FIXEDWING_FIRMWARE)
  if (autopilot_get_mode() == AP_MODE_AUTO2)
  {
    // Lateral XY coordinates
    lateral_mode = LATERAL_MODE_ROLL;

    float gvf_throttle_cmd = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE + step_throttle_controller(throttle_mod);
    guidance_v_SetCruiseThrottle(gvf_throttle_cmd);
    // printf("Current throttle: %.3f [%f ; %f]\n", v_ctl_auto_throttle_cruise_throttle, v_ctl_auto_throttle_min_cruise_throttle, v_ctl_auto_throttle_max_cruise_throttle);

    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    h_ctl_roll_setpoint =
        -gvf_parametric_control.k_roll * atanf(heading_rate * ground_speed / GVF_PARAMETRIC_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint); // Setting point for roll angle
  }
// Allow for rover operation
#elif defined(ROVER_FIRMWARE)
#else
#error gvf_parametric does not support your firmware yet
#endif
}

void gvf_parametric_low_level_control_3d(float heading_rate, float climbing_rate, float throttle_mod)
{
#if defined(FIXEDWING_FIRMWARE)
  if (autopilot_get_mode() == AP_MODE_AUTO2)
  {
    // Vertical Z coordinate
    v_ctl_mode = V_CTL_MODE_AUTO_CLIMB;
    v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;

    float gvf_throttle_cmd = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE + step_throttle_controller(throttle_mod);
    guidance_v_SetCruiseThrottle(gvf_throttle_cmd);
    // printf("Current throttle: %.3f [%f ; %f]\n", v_ctl_auto_throttle_cruise_throttle, v_ctl_auto_throttle_min_cruise_throttle, v_ctl_auto_throttle_max_cruise_throttle);

    v_ctl_climb_setpoint = gvf_parametric_control.k_climb * climbing_rate; // Setting point for vertical speed

    // Lateral XY coordinates
    lateral_mode = LATERAL_MODE_ROLL;

    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    h_ctl_roll_setpoint =
        -gvf_parametric_control.k_roll * atanf(heading_rate * ground_speed / GVF_PARAMETRIC_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint); // Setting point for roll angle
  }
// Allow for rover operation
#elif defined(ROVER_FIRMWARE)
#else
#error gvf_parametric does not support your firmware yet
#endif
}
