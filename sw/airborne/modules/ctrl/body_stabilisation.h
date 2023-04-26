/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file "modules/ctrl/body_stabilisation.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Module for stabilising the fuselage of a horizontal tiltwing
 */

#ifndef BODY_STABILISATION_H
#define BODY_STABILISATION_H

#include "std.h"

struct BodyStab {
  bool enabled;
  float kp_m;
  float ki_m;
  float kd_m;
  int u_eq_m;
  float kp_e;
  float ki_e;
  float kd_e;
  float state_integrator;
  float clamp_integrator;
  float airspeed_lb;
  float airspeed_ub;
  bool discrete_state; // 0 -> use motor, 1 -> use elevon
};

extern struct BodyStab body_stab;
extern struct FloatEulers euler_fus;
extern float motor_cmd;
extern float elevator_cmd;
extern float angle_wing2fus;



extern void body_stabilisation_init(void);
extern void body_stabilisation_periodic(void);
extern void body_stabilisation_report(void);

/**
 * settings handlers
 */
extern void body_stabilisation_reset(float enabled);
extern void body_stabilisation_update_kp_m(float kp);
extern void body_stabilisation_update_ki_m(float ki);
extern void body_stabilisation_update_kd_m(float kd);
extern void body_stabilisation_update_ueq_m(int ueq);

extern void body_stabilisation_update_kp_e(float kp);
extern void body_stabilisation_update_ki_e(float ki);
extern void body_stabilisation_update_kd_e(float kd);

extern void body_stabilisation_update_clamp_int(float clamp_val);
extern void body_stabilisation_update_airspeed_lb(float lb);
extern void body_stabilisation_update_airspeed_ub(float ub);

#endif  // BODY_STABILISATION_H
