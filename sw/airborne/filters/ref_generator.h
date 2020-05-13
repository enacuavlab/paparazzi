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

/** @file filters/ref_generator.h
 *  @brief Basic reference trajectory generator for attitude or guidance control loops
 *
 */

#ifndef REF_GENERATOR_H
#define REF_GENERATOR_H


/** Second order ref with saturations
 *  floating point
 */
struct RefTraj2_f {
  float setpoint;   ///< input position setpoint
  float pos;        ///< ref position
  float speed;      ///< ref speed
  float accel;      ///< ref accel
  float dt;         ///< integration step
  float max_speed;  ///< speed saturation
  float max_accel;  ///< accel saturation
  float w0_2;       ///< natural pulsation of the second order (squared)
  float damp;       ///< damping coefficient 2*w0*xi
};

/** Init second order ref traj (float)
 */
static inline void init_ref_traj2_f(struct RefTraj2_f *ref, float max_speed, float max_accel, float rise_time, float dt)
{
  ref->setpoint = 0.f;
  ref->pos = 0.f;
  ref->speed = 0.f;
  ref->accel = 0.f;
  ref->dt = dt;
  ref->max_speed = max_speed;
  ref->max_accel = max_accel;
  float w0 = 3.f / rise_time; // rise_time = 3*tau and tau = 1 / xi*w0 with xi = 1
  ref->w0_2 = w0 * w0;
  ref->damp = 2.f * w0; // with xi = 1
}

/** Set position of ref traj
 */
static inline void set_pos_ref_traj2_f(struct RefTraj2_f *ref, float pos, float speed)
{
  ref->setpoint = pos;
  ref->pos = pos;
  ref->speed = speed;
  ref->accel = 0.f;
}

/** Set rise time of ref traj
 */
static inline void set_rise_time_ref_traj2_f(struct RefTraj2_f *ref, float rise_time)
{
  float w0 = 3.f / rise_time; // rise_time = 3*tau and tau = 1 / xi*w0 with xi = 1
  ref->w0_2 = w0 * w0;
  ref->damp = 2.f * w0; // with xi = 1
}

/** Update reference trajectory
 */
static inline void update_ref_traj2_f(struct RefTraj2_f *ref, float setpoint)
{
  ref->setpoint = setpoint;
  ref->pos += ref->speed * ref->dt;
  ref->speed += ref->accel * ref->dt;
  ref->accel = ref->w0_2 * (ref->setpoint - ref->pos) - ref->damp * ref->speed;
  // saturation on accel
  if (ref->max_accel > 0.f) {
    if (ref->accel < -ref->max_accel) {
      ref->accel = -ref->max_accel;
    } else if (ref->accel > ref->max_accel) {
      ref->accel = ref->max_accel;
    }
  }
  // saturation on speed
  if (ref->max_speed > 0.f) {
    if (ref->speed < -ref->max_speed) {
      ref->speed = -ref->max_speed;
      // stop acceleration if speed is reached
      if (ref->accel < 0.f) {
        ref->accel = 0.f;
      }
    } else if (ref->speed > ref->max_speed) {
      ref->speed = ref->max_speed;
      // stop acceleration if speed is reached
      if (ref->accel > 0.f) {
        ref->accel = 0.f;
      }
    }
  }
}

#endif

