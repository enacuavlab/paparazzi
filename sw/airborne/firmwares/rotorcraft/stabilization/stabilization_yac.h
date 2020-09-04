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
 * @file firmwares/rotorcraft/stabilization/stabilization_yac.h
 *
 * Rotorcraft YAC attitude control.
 *
 */

#ifndef STABILIZATION_YAC_H
#define STABILIZATION_YAC_H

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "modules/ctrl/ctrl_yac.h"
#include "filters/ref_generator.h"
#include "generated/airframe.h"

struct StabYac_Rotorcraft {
  struct FloatEulers att_sp;    ///< attitude setpoint
  //float max_roll;               ///< max roll angle
  //float max_pitch;              ///< max pitch angle
  struct CtrlYac roll_ctrl;     ///< roll yac control
  struct RefTraj2_f roll_ref;   ///< roll ref trajectory
  struct CtrlYac pitch_ctrl;    ///< pitch yac control
  struct RefTraj2_f pitch_ref;  ///< pitch ref trajectory
  struct CtrlYac yaw_ctrl;      ///< yaw yac control
  struct RefTraj2_f yaw_ref;    ///< yaw ref trajectory
  int32_t cmd[COMMANDS_NB];     ///< command vector
};

extern struct StabYac_Rotorcraft stab_yac;

extern void stab_yac_init(void);
extern void stab_yac_attitude_run(bool in_flight);
extern void stab_yac_attitude_enter(void);
extern void stab_yac_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern void stab_yac_attitude_set_earth_cmd(struct Int32Vect2 *cmd, float heading);

#endif /* STABILIZATION_YAC_H */

