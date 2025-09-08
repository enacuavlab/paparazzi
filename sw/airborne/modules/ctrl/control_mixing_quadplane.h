/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/ctrl/control_mixing_quadplane.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing for quadplane
 */

#ifndef CONTROL_MIXING_QUADPLANE_H
#define CONTROL_MIXING_QUADPLANE_H

// INDI actuators output indexes
#define CMQ_ACT_MOTOR_FRONT_RIGHT 0
#define CMQ_ACT_MOTOR_BACK_RIGHT  1
#define CMQ_ACT_MOTOR_BACK_LEFT   2
#define CMQ_ACT_MOTOR_FRONT_LEFT  3

extern void control_mixing_quadplane_init(void);

/** Direct manual control in plane style flight
 */
extern void control_mixing_quadplane_manual(void);

/** Stabilization in attitude direct mode
 */
extern void control_mixing_quadplane_attitude_direct(void);
extern void control_mixing_quadplane_attitude_direct_enter(void);
extern void control_mixing_quadplane_attitude_plane(void);
extern void control_mixing_quadplane_attitude_plane_enter(void);
extern void control_mixing_quadplane_nav_enter(void);
extern void control_mixing_quadplane_nav_run(void);

#endif  // CONTROL_MIXING_QUADPLANE_H
