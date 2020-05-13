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
 * @file modules/ctrl/ctrl_yac.c
 *
 * YAC control lib
 */

#include "modules/ctrl/ctrl_yac.h"
#include "paparazzi.h"

void ctrl_yac_init(struct CtrlYac *yac, float kp, float kd, float scaling, float beta, float mincutoff, float freq)
{
  yac->kp = kp;
  yac->kd = kd;
  yac->scaling = scaling;
  yac->model_est = 0.f;
  yac->command = 0.f;
  init_1e_filter(&yac->filter, freq, mincutoff, beta, 1.f);
}

void ctrl_yac_reset(struct CtrlYac *yac)
{
  yac->model_est = 0.f;
  yac->command = 0.f;
  reset_1e_filter(&yac->filter);
}

float ctrl_yac_compute_command(struct CtrlYac *yac, float input, float d_input, float ref, float d_ref)
{
  float err = input - ref;
  float d_err = d_input - d_ref;
  float pd_feedback = yac->kp * err + yac->kd * d_err;
  yac->model_est = update_1e_filter(&yac->filter, d_input - (yac->scaling * yac->command / MAX_PPRZ));
  yac->command = - MAX_PPRZ * (yac->model_est - d_ref + pd_feedback) / yac->scaling;
  return yac->command;
}

