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
 * @file firmwares/fixedwing/stabilization/stabilization_yac.h
 *
 * Fixed wing horizontal control.
 *
 */

#ifndef STABILIZATION_YAC_H
#define STABILIZATION_YAC_H

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "modules/ctrl/ctrl_yac.h"
#include "filters/ref_generator.h"
#include "generated/airframe.h"

struct StabYac_FW {
  struct FloatEulers att_sp;    ///< attitude setpoint
  float max_roll;               ///< max roll angle
  float max_pitch;              ///< max pitch angle
  struct CtrlYac roll_ctrl;     ///< roll yac control
  struct RefTraj2_f roll_ref;   ///< roll ref trajectory
  pprz_t roll_cmd;              ///< roll command
  struct CtrlYac pitch_ctrl;    ///< pitch yac control
  struct RefTraj2_f pitch_ref;  ///< roll ref trajectory
  pprz_t pitch_cmd;             ///< pitch command
};

extern struct StabYac_FW stab_yac;

extern void stab_yac_init(void);
extern void stab_yac_attitude_loop(void);
extern void stab_yac_set_attitude_setpoint(float roll, float pitch);
extern pprz_t stab_yac_get_roll_cmd(void);
extern pprz_t stab_yac_get_pitch_cmd(void);

#endif /* STABILIZATION_YAC_H */
