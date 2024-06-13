/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_point_mass.c
 *
 */
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_point_mass.h"
#include "filters/low_pass_filter.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "autopilot.h"

#ifndef GUIDANCE_POINT_MASS_SPEED_TAU
#define GUIDANCE_POINT_MASS_SPEED_TAU 1.
#endif

#ifndef GUIDANCE_POINT_MASS_HEADING_TAU
#define GUIDANCE_POINT_MASS_HEADING_TAU 1.
#endif

#ifndef GUIDANCE_POINT_MASS_VZ_TAU
#define GUIDANCE_POINT_MASS_VZ_TAU 1.
#endif

float point_mass_speed_tau;
float point_mass_heading_tau;
float point_mass_vz_tau;

static struct FirstOrderLowPass lp_speed;
static struct FirstOrderLowPass lp_heading;
static struct FirstOrderLowPass lp_vz;

/**
 * @brief Init function
 */
void guidance_point_mass_init(void)
{
  point_mass_speed_tau = GUIDANCE_POINT_MASS_SPEED_TAU;
  point_mass_heading_tau = GUIDANCE_POINT_MASS_HEADING_TAU;
  point_mass_vz_tau = GUIDANCE_POINT_MASS_VZ_TAU;

  init_first_order_low_pass(&lp_speed, point_mass_speed_tau, (1.f / PERIODIC_FREQUENCY), 0.f);
  init_first_order_low_pass(&lp_heading, point_mass_heading_tau, (1.f / PERIODIC_FREQUENCY), 0.f);
  init_first_order_low_pass(&lp_vz, point_mass_vz_tau, (1.f / PERIODIC_FREQUENCY), 0.f);
}

/**
 *
 * Call upon entering
 */
void guidance_point_mass_enter(void)
{
  // TODO reset ?
}

/**
 * Run point pass model
 */
void guidance_point_mass_h_run(bool in_flight UNUSED, struct HorizontalGuidance *gh UNUSED)
{
  // propagate point mass model
  // update state
}

void guidance_point_mass_v_run(bool in_flight UNUSED, struct VerticalGuidance *gv UNUSED)
{
  // propagate point mass model
  // update state
  switch (gv->mode) {
    case GUIDANCE_V_MODE_KILL:
    case GUIDANCE_V_MODE_RC_DIRECT:
    case GUIDANCE_V_MODE_RC_CLIMB:
      // TODO implement free fall ? :)
      break;
    case GUIDANCE_V_MODE_CLIMB:
      // TODO
      break;

}

#if GUIDANCE_POINT_MASS_USE_AS_DEFAULT
// guidance indi control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_point_mass_enter();
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  guidance_point_mass_h_run(in_flight, gh);
  struct StabilizationSetpoint sp;
  STAB_SP_SET_EULERS_ZERO(sp);
  return sp;
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_h_run_pos(in_flight, gh);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_h_run_pos(in_flight, gh);
}

struct ThrustSetpoint guidance_v_run_pos(bool in_flight, struct VerticalGuidance *gv)
{
  guidance_point_mass_v_run(in_flight, gv);
  struct ThrustSetpoint sp;
  THRUST_SP_SET_ZERO(sp);
  return sp;
}

struct ThrustSetpoint guidance_v_run_speed(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_v_run_pos(in_flight, gv);
}

struct ThrustSetpoint guidance_v_run_accel(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_v_run_pos(in_flight, gv);
}

#endif

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

void stabilization_attitude_run(UNUSED bool in_flight, UNUSED struct StabilizationSetpoint *sp, UNUSED struct ThrustSetpoint *thrust, UNUSED int32_t *cmd)
{
}

void stabilization_attitude_enter(void) {}

