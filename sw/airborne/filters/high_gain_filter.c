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

/** @file filters/high_gain_filter.c
 *  @brief Implementation of the high gain filter for rotary encoder
 */

#include "filters/high_gain_filter.h"
#include "math/pprz_algebra_float.h"


/** Init all matrix and vectors to the right value
 *
 * @param filter pointer to a filter structure
 * @param kp proportional gain
 * @param kv derivative gain
 * @param epsilon high gain
 * @param rate data update rate
 */
void high_gain_filter_init(struct high_gain_filter *filter, float kp, float kv, float epsilon, float rate,  float init_val_theta, float init_val_theta_dot){
  filter->kp = kp;
  filter->kv = kv;
  filter->epsilon = epsilon;
  filter->rate = rate;

  filter->hatx[0] = init_val_theta;
  filter->hatx[1] = init_val_theta_dot;
  filter->hatx_dot_prev[0] = 0;
  filter->hatx_dot_prev[1] = 0;
}


/** Process step
 *
 * hatx_dot =  A * hatx + B * (theta - hatx[0])
 * hatx += (1/rate)* (hatx_dot_prev + hatx)/2
 *
 * @param filter pointer to the filter structure
 * @param theta measurement value
 */

void high_gain_filter_process(struct high_gain_filter *filter, float theta){
  float hatx_dot[2] = {filter->hatx[1] + (filter->kp/filter->epsilon)*(theta-filter->hatx[0]), (filter->kv/filter->epsilon)*(theta-filter->hatx[0])};
  filter->hatx[0] += (1/filter->rate)*(filter->hatx_dot_prev[0] + hatx_dot[0])/2;
  filter->hatx[1] += (1/filter->rate)*(filter->hatx_dot_prev[1] + hatx_dot[1])/2;
  filter->hatx_dot_prev[0] = hatx_dot[0];
  filter->hatx_dot_prev[1] = hatx_dot[1];
}