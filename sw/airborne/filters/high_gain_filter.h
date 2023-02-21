/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file filters/high_gain_filter.h
 *  @brief Implementation of the high gain filter for rotary encoder
 */

#pragma once

#include "std.h"
#include <math.h>


/**
 *  filter struct
 */
struct high_gain_filter {

  //states
  float hatx[3];  // state 
  float hatx_dot_prev[3]; // previous state

  //parameters
  float alpha[3];
  float epsilon;                     //
  float rate;                     ///< data update rate (in Hz)

};

/** Init all matrix and vectors to the right value
 *
 * @param filter pointer to a filter structure
 * @param alpha 
 * @param epsilon high gain
 * @param rate data update rate
 * @param init_val_theta
 * @param init_val_theta_dot
 */
extern void high_gain_filter_init(struct high_gain_filter *filter, float *alpha, float epsilon, float rate);

/** Process step
 *
 * hatx_dot =  A * hatx + B * (theta - hatx[0])
 * hatx += (1/rate)* (hatxprev + hatx)/2
 *
 * @param filter pointer to the filter structure
 * @param theta measurement value
 */
extern void high_gain_filter_process(struct high_gain_filter *filter, float theta);