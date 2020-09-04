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
 * @file firmwares/rotorcraft/stabilization/stabilization_yac.c
 *
 * Rotorcraft YAC attitude control.
 *
 */

#include "firmwares/rotorcraft/stabilization/stabilization_yac.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "state.h"
#include "autopilot.h"

#include <stdio.h>

// commands are already prescaled by MAX_PPRZ, so a scaling of 1 is the default
#ifndef STABILIZATION_YAC_ROLL_SCALING
#define STABILIZATION_YAC_ROLL_SCALING 1.f
#endif

// commands are already prescaled by MAX_PPRZ, so a scaling of 1 is the default
#ifndef STABILIZATION_YAC_PITCH_SCALING
#define STABILIZATION_YAC_PITCH_SCALING 1.f
#endif

// commands are already prescaled by MAX_PPRZ, so a scaling of 1 is the default
#ifndef STABILIZATION_YAC_YAW_SCALING
#define STABILIZATION_YAC_YAW_SCALING 1.f
#endif

#ifndef STABILIZATION_YAC_REF_ROLL_RISE_TIME
#define STABILIZATION_YAC_REF_ROLL_RISE_TIME 0.2f
#endif

#ifndef STABILIZATION_YAC_REF_PITCH_RISE_TIME
#define STABILIZATION_YAC_REF_PITCH_RISE_TIME STABILIZATION_YAC_REF_ROLL_RISE_TIME
#endif

#ifndef STABILIZATION_YAC_REF_YAW_RISE_TIME
#define STABILIZATION_YAC_REF_YAW_RISE_TIME 0.5f
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

#ifndef STABILIZATION_YAC_YAW_BETA
#define STABILIZATION_YAC_YAW_BETA STABILIZATION_YAC_ROLL_BETA
#endif

#ifndef STABILIZATION_YAC_YAW_MINCUTOFF
#define STABILIZATION_YAC_YAW_MINCUTOFF STABILIZATION_YAC_ROLL_MINCUTOFF
#endif

//#ifndef STABILIZATION_YAC_MAX_ROLL
//#define STABILIZATION_YAC_MAX_ROLL RadOfDeg(45.f)
//#endif
//
//#ifndef STABILIZATION_YAC_MAX_PITCH
//#define STABILIZATION_YAC_MAX_PITCH RadOfDeg(45.f)
//#endif

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

// max ref yaw rate unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_R
#define STABILIZATION_YAC_REF_MAX_R -1.f
#endif

// max ref yaw acceleration unlimited by default
#ifndef STABILIZATION_YAC_REF_MAX_R_DOT
#define STABILIZATION_YAC_REF_MAX_R_DOT -1.f
#endif


static const float ctrl_yac_dt = (1.f / PERIODIC_FREQUENCY);

struct StabYac_Rotorcraft stab_yac;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ctl_a(struct transport_tx *trans, struct link_device *dev)
{
  pprz_t cmd_roll = stab_yac.cmd[COMMAND_ROLL];
  pprz_t cmd_pitch = stab_yac.cmd[COMMAND_PITCH];
  pprz_msg_send_H_CTL_A(trans, dev, AC_ID,
                        &stab_yac.roll_ctrl.model_est,
                        &stab_yac.roll_ref.setpoint,
                        &stab_yac.roll_ref.pos,
                        &(stateGetNedToBodyEulers_f()->phi),
                        &cmd_roll,
                        &stab_yac.pitch_ctrl.model_est,
                        &stab_yac.pitch_ref.setpoint,
                        &stab_yac.pitch_ref.pos,
                        &(stateGetNedToBodyEulers_f()->theta),
                        &cmd_pitch);
}
#endif

void stab_yac_init(void)
{
  FLOAT_EULERS_ZERO(stab_yac.att_sp);
//  stab_yac.max_roll = STABILIZATION_YAC_MAX_ROLL;
//  stab_yac.max_pitch = STABILIZATION_YAC_MAX_PITCH;
  ctrl_yac_init(&stab_yac.roll_ctrl,
      STABILIZATION_YAC_ROLL_KP,
      STABILIZATION_YAC_ROLL_KD,
      STABILIZATION_YAC_ROLL_SCALING,
      STABILIZATION_YAC_ROLL_BETA,
      STABILIZATION_YAC_ROLL_MINCUTOFF,
      PERIODIC_FREQUENCY);
  init_ref_traj2_f(&stab_yac.roll_ref,
      STABILIZATION_YAC_REF_MAX_P,
      STABILIZATION_YAC_REF_MAX_P_DOT,
      STABILIZATION_YAC_REF_ROLL_RISE_TIME,
      ctrl_yac_dt);
  ctrl_yac_init(&stab_yac.pitch_ctrl,
      STABILIZATION_YAC_PITCH_KP,
      STABILIZATION_YAC_PITCH_KD,
      STABILIZATION_YAC_PITCH_SCALING,
      STABILIZATION_YAC_PITCH_BETA,
      STABILIZATION_YAC_PITCH_MINCUTOFF,
      PERIODIC_FREQUENCY);
  init_ref_traj2_f(&stab_yac.pitch_ref,
      STABILIZATION_YAC_REF_MAX_Q,
      STABILIZATION_YAC_REF_MAX_Q_DOT,
      STABILIZATION_YAC_REF_PITCH_RISE_TIME,
      ctrl_yac_dt);
  ctrl_yac_init(&stab_yac.yaw_ctrl,
      STABILIZATION_YAC_YAW_KP,
      STABILIZATION_YAC_YAW_KD,
      STABILIZATION_YAC_YAW_SCALING,
      STABILIZATION_YAC_YAW_BETA,
      STABILIZATION_YAC_YAW_MINCUTOFF,
      PERIODIC_FREQUENCY);
  init_ref_traj2_f(&stab_yac.pitch_ref,
      STABILIZATION_YAC_REF_MAX_Q,
      STABILIZATION_YAC_REF_MAX_Q_DOT,
      STABILIZATION_YAC_REF_PITCH_RISE_TIME,
      ctrl_yac_dt);
  for (int i = 0; i < COMMANDS_NB; i++) {
    stab_yac.cmd[i] = 0;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_H_CTL_A, send_ctl_a);
#endif
}


static void stab_yac_roll_loop(bool in_flight)
{
  update_ref_traj2_f(&stab_yac.roll_ref, stab_yac.att_sp.phi);
  ctrl_yac_compute_command(&stab_yac.roll_ctrl,
      stateGetNedToBodyEulers_f()->phi,
      stateGetBodyRates_f()->p,
      stab_yac.roll_ref.pos,
      stab_yac.roll_ref.speed);
  if (!in_flight) {
    ctrl_yac_reset(&stab_yac.roll_ctrl);
  }
  BoundAbs(stab_yac.roll_ctrl.command, MAX_PPRZ);
  stab_yac.cmd[COMMAND_ROLL] = stab_yac.roll_ctrl.command;
}


static void stab_yac_pitch_loop(bool in_flight)
{
  update_ref_traj2_f(&stab_yac.pitch_ref, stab_yac.att_sp.theta);
  ctrl_yac_compute_command(&stab_yac.pitch_ctrl,
      stateGetNedToBodyEulers_f()->theta,
      stateGetBodyRates_f()->q,
      stab_yac.pitch_ref.pos,
      stab_yac.pitch_ref.speed);
  if (!in_flight) {
    ctrl_yac_reset(&stab_yac.pitch_ctrl);
  }
  BoundAbs(stab_yac.pitch_ctrl.command, MAX_PPRZ);
  stab_yac.cmd[COMMAND_PITCH] = stab_yac.pitch_ctrl.command;
}

static void stab_yac_yaw_loop(bool in_flight)
{
  update_ref_traj2_f(&stab_yac.yaw_ref, stab_yac.att_sp.psi);
  ctrl_yac_compute_command(&stab_yac.yaw_ctrl,
      stateGetNedToBodyEulers_f()->psi,
      stateGetBodyRates_f()->r,
      stab_yac.yaw_ref.pos,
      stab_yac.yaw_ref.speed);
  if (!in_flight) {
    ctrl_yac_reset(&stab_yac.yaw_ctrl);
  }
  BoundAbs(stab_yac.yaw_ctrl.command, MAX_PPRZ);
  stab_yac.cmd[COMMAND_YAW] = stab_yac.yaw_ctrl.command;
}


void stab_yac_attitude_run(bool in_flight)
{
  stab_yac_roll_loop(in_flight);
  stab_yac_pitch_loop(in_flight);
  stab_yac_yaw_loop(in_flight);
  //printf("%d %d %d %d\n", stab_yac.cmd[COMMAND_ROLL], stab_yac.cmd[COMMAND_PITCH], stab_yac.cmd[COMMAND_YAW], stab_yac.cmd[COMMAND_THRUST]);
}

void stab_yac_attitude_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_yac.att_sp.psi = stabilization_attitude_get_heading_f();
  set_pos_ref_traj2_f(&stab_yac.yaw_ref, stab_yac.att_sp.psi, 0.f);
}

void stab_yac_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_yac.att_sp, in_flight, in_carefree, coordinated_turn);
}

void stab_yac_attitude_set_earth_cmd(struct Int32Vect2 *cmd, float heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_yac.att_sp.psi = heading;

  // compute sp_euler phi/theta
  // Rotate horizontal commands to body frame by psi
  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sinf(psi);
  float c_psi = cosf(psi);
  float cmd_x = ANGLE_FLOAT_OF_BFP(cmd->x);
  float cmd_y = ANGLE_FLOAT_OF_BFP(cmd->y);
  stab_yac.att_sp.phi = (-s_psi * cmd_x + c_psi * cmd_y);
  stab_yac.att_sp.theta = -(c_psi * cmd_x + s_psi * cmd_y);
  //printf("%f %f | %f %f | %f\n",stab_yac.att_sp.phi, cmd_x, stab_yac.att_sp.theta, cmd_y, stab_yac.att_sp.psi);
}


