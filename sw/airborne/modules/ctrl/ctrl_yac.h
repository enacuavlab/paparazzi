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
 * @file modules/ctrl/ctrl_yac.h
 *
 * YAC control lib
 */

#ifndef CTRL_YAC_H
#define CTRL_YAC_H

#include "std.h"
#include "filters/1e_filter.h"

/**
 * YAC controller structure
 */
struct CtrlYac {
  float kp;                     ///< proportional gain
  float kd;                     ///< derivative gain
  float scaling;                ///< scaling factor
  float model_est;              ///< estimated model error
  float command;                ///< last computed command
  struct OneEuroFilter filter;  ///< one euro filter for model estimation
};

/**
 * Init YAC controller
 *
 * @param yac pointer to YAC structure to initialized
 * @param kp proportional gain
 * @param kd derivative gain
 * @param scaling scaling factor
 * @param beta adaptation factor of 1e filter
 * @param mincutoff minimum cutoff frequency of 1e filter
 * @param rate update rate in Hz
 */
void ctrl_yac_init(struct CtrlYac *yac, float kp, float kd, float scaling, float beta, float mincutoff, float freq);

/**
 * Reset YAC controller
 *
 * @param yac pointer to YAC controller
 */
void ctrl_yac_reset(struct CtrlYac *yac);

/**
 * Conpute next command
 * Should be called at the frequency specified in the controller structure
 *
 * @param yac pointer to YAC controller
 * @param input input signal
 * @param d_input first derivative of the input signal
 * @param ref reference setpoint to be tracked
 * @param d_ref first derivative of the reference setpoint
 * @return computed command
 */
float ctrl_yac_compute_command(struct CtrlYac *yac, float input, float d_input, float ref, float d_ref);

#endif

