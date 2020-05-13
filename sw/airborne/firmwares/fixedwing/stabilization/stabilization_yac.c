/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/fixedwing/stabilization/stabilization_yac.c
 *
 * Fixed wing horizontal control.
 *
 */

#include "firmwares/fixedwing/stabilization/stabilization_yac.h"
#include "state.h"
#include "autopilot.h"

// commands are already prescaled by MAX_PPRZ, so a scaling of 1 is the default
#ifndef STABILIZATION_YAC_ROLL_SCALING
#define STABILIZATION_YAC_ROLL_SCALING 1.f
#endif

// commands are already prescaled by MAX_PPRZ, so a scaling of 1 is the default
#ifndef STABILIZATION_YAC_PITCH_SCALING
#define STABILIZATION_YAC_PITCH_SCALING 1.f
#endif

#ifndef STABILIZATION_YAC_REF_ROLL_RISE_TIME
#define STABILIZATION_YAC_REF_ROLL_RISE_TIME 0.5f
#endif

#ifndef STABILIZATION_YAC_REF_PITCH_RISE_TIME
#define STABILIZATION_YAC_REF_PITCH_RISE_TIME STABILIZATION_YAC_REF_ROLL_RISE_TIME
#endif

#ifndef STABILIZATION_YAC_ROLL_BETA
#define STABILIZATION_YAC_ROLL_BETA 0.01f
#endif

#ifndef STABILIZATION_YAC_ROLL_MINCUTOFF
#define STABILIZATION_YAC_ROLL_MINCUTOFF 0.1f
#endif

#ifndef STABILIZATION_YAC_PITCH_BETA
#define STABILIZATION_YAC_PITCH_BETA STABILIZATION_YAC_ROLL_BETA
#endif

#ifndef STABILIZATION_YAC_PITCH_MINCUTOFF
#define STABILIZATION_YAC_PITCH_MINCUTOFF STABILIZATION_YAC_ROLL_MINCUTOFF
#endif

// default max bank angle of 40 deg (1.3 load factor)
#ifndef STABILIZATION_YAC_MAX_ROLL
#define STABILIZATION_YAC_MAX_ROLL RadOfDeg(40.f)
#endif

// default max pitch angle of 30 deg
#ifndef STABILIZATION_YAC_MAX_PITCH
#define STABILIZATION_YAC_MAX_PITCH RadOfDeg(30.f)
#endif

// max ref roll rate unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_P
#define STABILIZATION_YAC_REF_MAX_P -1.f
#endif

// max ref roll acceleration unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_P_DOT
#define STABILIZATION_YAC_REF_MAX_P_DOT -1.f
#endif

// max ref pitch rate unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_Q
#define STABILIZATION_YAC_REF_MAX_Q -1.f
#endif

// max ref pitch acceleration unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_Q_DOT
#define STABILIZATION_YAC_REF_MAX_Q_DOT -1.f
#endif


static const float ctrl_yac_dt = (1.f / CONTROL_FREQUENCY);

struct StabYac_FW stab_yac;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_tune_roll(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TUNE_ROLL(trans, dev, AC_ID,
                          &(stateGetBodyRates_f()->p), &(stateGetNedToBodyEulers_f()->phi), &stab_yac.att_sp.phi);
}

static void send_ctl_a(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_H_CTL_A(trans, dev, AC_ID,
                        &stab_yac.roll_ctrl.model_est,
                        &stab_yac.roll_ref.setpoint,
                        &stab_yac.roll_ref.pos,
                        &(stateGetNedToBodyEulers_f()->phi),
                        &stab_yac.roll_cmd,
                        &stab_yac.pitch_ctrl.model_est,
                        &stab_yac.pitch_ref.setpoint,
                        &stab_yac.pitch_ref.pos,
                        &(stateGetNedToBodyEulers_f()->theta),
                        &stab_yac.pitch_cmd);
}
#endif

void stab_yac_init(void)
{
  FLOAT_EULERS_ZERO(stab_yac.att_sp);
  stab_yac.max_roll = STABILIZATION_YAC_MAX_ROLL;
  stab_yac.max_pitch = STABILIZATION_YAC_MAX_PITCH;
  ctrl_yac_init(&stab_yac.roll_ctrl,
      STABILIZATION_YAC_ROLL_KP,
      STABILIZATION_YAC_ROLL_KD,
      STABILIZATION_YAC_ROLL_SCALING,
      STABILIZATION_YAC_ROLL_BETA,
      STABILIZATION_YAC_ROLL_MINCUTOFF,
      CONTROL_FREQUENCY);
  init_ref_traj2_f(&stab_yac.roll_ref,
      STABILIZATION_YAC_REF_MAX_P,
      STABILIZATION_YAC_REF_MAX_P_DOT,
      STABILIZATION_YAC_REF_ROLL_RISE_TIME,
      ctrl_yac_dt);
  stab_yac.roll_cmd = 0;
  ctrl_yac_init(&stab_yac.pitch_ctrl,
      STABILIZATION_YAC_PITCH_KP,
      STABILIZATION_YAC_PITCH_KD,
      STABILIZATION_YAC_PITCH_SCALING,
      STABILIZATION_YAC_PITCH_BETA,
      STABILIZATION_YAC_PITCH_MINCUTOFF,
      CONTROL_FREQUENCY);
  init_ref_traj2_f(&stab_yac.pitch_ref,
      STABILIZATION_YAC_REF_MAX_Q,
      STABILIZATION_YAC_REF_MAX_Q_DOT,
      STABILIZATION_YAC_REF_PITCH_RISE_TIME,
      ctrl_yac_dt);
  stab_yac.pitch_cmd = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TUNE_ROLL, send_tune_roll);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_H_CTL_A, send_ctl_a);
#endif
}


static void stab_yac_roll_loop(void)
{
  update_ref_traj2_f(&stab_yac.roll_ref, stab_yac.att_sp.phi);
  ctrl_yac_compute_command(&stab_yac.roll_ctrl,
      stateGetNedToBodyEulers_f()->phi,
      stateGetBodyRates_f()->p,
      stab_yac.roll_ref.pos,
      stab_yac.roll_ref.speed);
  if (!autopilot.launch) {
    ctrl_yac_reset(&stab_yac.roll_ctrl);
  }
  BoundAbs(stab_yac.roll_ctrl.command, MAX_PPRZ);
  stab_yac.roll_cmd = (pprz_t) stab_yac.roll_ctrl.command;
}


static void stab_yac_pitch_loop(void)
{
  update_ref_traj2_f(&stab_yac.pitch_ref, stab_yac.att_sp.theta);
  // TODO add an offset to pitch rate setpoint as a function of desired turn rate
  //float nz_c = 1.f / cosf(stab_yac.roll_ref.pos);
  //float q_turn = (9.81f / 15.) * (nz_c - 1.f/nz_c);
  //float t_turn = (nz_c - 1) / 3.f;
  ctrl_yac_compute_command(&stab_yac.pitch_ctrl,
      stateGetNedToBodyEulers_f()->theta,
      stateGetBodyRates_f()->q,
      stab_yac.pitch_ref.pos,
      stab_yac.pitch_ref.speed);
      //stab_yac.pitch_ref.pos + t_turn,
      //stab_yac.pitch_ref.speed + q_turn);
  if (!autopilot.launch) {
    ctrl_yac_reset(&stab_yac.pitch_ctrl);
  }
  BoundAbs(stab_yac.pitch_ctrl.command, MAX_PPRZ);
  stab_yac.pitch_cmd = (pprz_t) stab_yac.pitch_ctrl.command;
}


void stab_yac_attitude_loop(void)
{
  stab_yac_roll_loop();
  stab_yac_pitch_loop();
}

void stab_yac_set_attitude_setpoint(float roll, float pitch)
{
  stab_yac.att_sp.phi = roll;
  BoundAbs(stab_yac.att_sp.phi, stab_yac.max_roll);
  stab_yac.att_sp.theta = pitch;
  BoundAbs(stab_yac.att_sp.theta, stab_yac.max_pitch);
}

pprz_t stab_yac_get_roll_cmd(void)
{
  return stab_yac.roll_cmd;
}

pprz_t stab_yac_get_pitch_cmd(void)
{
  return stab_yac.pitch_cmd;
}

