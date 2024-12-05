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

/**
 * @file stabilization_attitude_plane_pid.c
 *
 * Basic fixed-wing attitude stabilization in euler float version.
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_plane_pid.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

struct FloatAttitudeGains stab_plane_gains;
struct FloatEulers stab_plane_att_sum_err;

static struct FloatEulers stab_att_sp_euler;
static struct AttRefEulerFloat att_ref_euler_f;

float stab_plane_att_fb_cmd[COMMANDS_NB];

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates *body_rate = stateGetBodyRates_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float foo = 0.0;
  pprz_msg_send_STAB_ATTITUDE_FLOAT(trans, dev, AC_ID,
                                    &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                    &(att->phi), &(att->theta), &(att->psi),
                                    &stab_att_sp_euler.phi,
                                    &stab_att_sp_euler.theta,
                                    &stab_att_sp_euler.psi,
                                    &stab_plane_att_sum_err.phi,
                                    &stab_plane_att_sum_err.theta,
                                    &stab_plane_att_sum_err.psi,
                                    &stabilization_att_fb_cmd[COMMAND_ROLL],
                                    &stabilization_att_fb_cmd[COMMAND_PITCH],
                                    &stabilization_att_fb_cmd[COMMAND_YAW],
                                    &stabilization_att_ff_cmd[COMMAND_ROLL],
                                    &stabilization_att_ff_cmd[COMMAND_PITCH],
                                    &stabilization_att_ff_cmd[COMMAND_YAW],
                                    &stabilization.cmd[COMMAND_ROLL],
                                    &stabilization.cmd[COMMAND_PITCH],
                                    &stabilization.cmd[COMMAND_YAW],
                                    &foo, &foo, &foo);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_REF_FLOAT(trans, dev, AC_ID,
                                        &stab_att_sp_euler.phi,
                                        &stab_att_sp_euler.theta,
                                        &stab_att_sp_euler.psi,
                                        &att_ref_euler_f.euler.phi,
                                        &att_ref_euler_f.euler.theta,
                                        &att_ref_euler_f.euler.psi,
                                        &att_ref_euler_f.rate.p,
                                        &att_ref_euler_f.rate.q,
                                        &att_ref_euler_f.rate.r,
                                        &att_ref_euler_f.accel.p,
                                        &att_ref_euler_f.accel.q,
                                        &att_ref_euler_f.accel.r);
}
#endif

void stabilization_attitude_euler_float_init(void)
{

  attitude_ref_euler_float_init(&att_ref_euler_f);

  VECT3_ASSIGN(stab_plane_gains.p,
               STABILIZATION_ATTITUDE_PHI_PGAIN,
               STABILIZATION_ATTITUDE_THETA_PGAIN,
               0.f);

  VECT3_ASSIGN(stab_plane_gains.d,
               STABILIZATION_ATTITUDE_PHI_DGAIN,
               STABILIZATION_ATTITUDE_THETA_DGAIN,
               0.f);

  VECT3_ASSIGN(stab_plane_gains.i,
               STABILIZATION_ATTITUDE_PHI_IGAIN,
               STABILIZATION_ATTITUDE_THETA_IGAIN,
               0.f);

  FLOAT_EULERS_ZERO(stab_plane_att_sum_err);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_FLOAT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_FLOAT, send_att_ref);
#endif
}

void stabilization_attitude_enter(void)
{
  /* reset psi ref to current psi angle */
  //attitude_ref_euler_float_enter(&att_ref_euler_f, stabilization_attitude_get_heading_f());

  FLOAT_EULERS_ZERO(stab_plane_att_sum_err);
}

#define MAX_SUM_ERR 200

void stabilization_attitude_plane_pid_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd);
{
  stab_att_sp_euler = stab_sp_to_eulers_f(sp);
  EULERS_COPY(att_ref_euler_f.euler, stab_att_sp_euler);
  FLOAT_RATES_ZERO(att_ref_euler_f.rate);

  /* Compute feedback        */
  /* attitude error          */
  struct FloatEulers *att_float = stateGetNedToBodyEulers_f();
  struct FloatEulers att_err;
  EULERS_DIFF(att_err, att_ref_euler_f.euler, *att_float);
  FLOAT_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(stab_plane_att_sum_err, att_err);
    EULERS_BOUND_CUBE(stab_plane_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  } else {
    FLOAT_EULERS_ZERO(stab_plane_att_sum_err);
  }

  /*  rate error             */
  struct FloatRates *rate_float = stateGetBodyRates_f();
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, att_ref_euler_f.rate, *rate_float);

  /*  PID                    */

  cmd[COMMAND_ROLL] =
    stab_plane_gains.p.x  * att_err.phi +
    stab_plane_gains.d.x  * rate_err.p +
    stab_plane_gains.i.x  * stab_plane_att_sum_err.phi;

  cmd[COMMAND_PITCH] =
    stab_plane_gains.p.y  * att_err.theta +
    stab_plane_gains.d.y  * rate_err.q +
    stab_plane_gains.i.y  * stab_plane_att_sum_err.theta;

  cmd[COMMAND_YAW] = 0;
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z); // TODO check that

  /* bound the result */
  BoundAbs(cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_THRUST], MAX_PPRZ);
}
