/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file firmwares/rotorcraft/guidance/guidance_pid.c
 *  Guidance controller with PID for rotorcrafts.
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_plane.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "state.h"

/*
 * external variables
 */

struct GuidancePlane guidance_plane;

/*
 * internal variables
 */


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

#endif

void guidance_plane_init(void)
{
  guidance_plane.course_sp = 0.f;
  guidance_plane.course_kp = GUIDANCE_PLANE_COURSE_KP;
  guidance_plane.course_kd = GUIDANCE_PLANE_COURSE_KD;
  guidance_plane.roll_sp = 0.f;

#if PERIODIC_TELEMETRY
#endif
}

/**
 * run horizontal control loop for position and speed control
 */
struct StabilizationSetpoint guidance_plane_h_run(bool in_flight, struct HorizontalGuidance *gh)
{
  struct FloatEulers att_sp;

  // course control loop


  return stab_sp_from_eulers_f(&att_sp);
}

/**
 * run vertical control loop for position and speed control
 */
struct ThrustSetpoint guidance_plane_v_run(bool in_flight, struct VerticalGuidance *gv)
{
  return th_sp_from_thrust_i(guidance_plane.thrust_cmd, THRUST_AXIS_Z);
}

void guidance_plane_enter(void)
{
  /* set nav_heading to current heading */
  //nav.heading = stateGetNedToBodyEulers_f()->psi;
  //guidance_pid_z_sum_err = 0;
}

#endif

