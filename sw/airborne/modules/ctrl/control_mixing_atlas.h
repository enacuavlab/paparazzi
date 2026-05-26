/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/ctrl/control_mixing_atlas.h"
 * Control mixing specifics for the ATLAS tilt-rotor
 */

#ifndef CONTROL_MIXING_ATLAS_H
#define CONTROL_MIXING_ATLAS_H

// // INDI actuators output indexes
// #define CMA_ACT_MOTOR_FR   0          // Motor Front-Right (FR)
// #define CMA_ACT_MOTOR_BR   1          // Motor Back-Right (RR)
// #define CMA_ACT_MOTOR_BL   2          // Motor Back-Left (RL)
// #define CMA_ACT_MOTOR_FL   3          // Motor Front-Left (FL)
// #define CMA_ACT_TILT_R     4          // Tilt Servo Right (TILT_R), 0 = vertical, pi/2 = forward
// #define CMA_ACT_TILT_L     5          // Tilt Servo Left (TILT_L), 0 = vertical, pi/2 = forward

extern void control_mixing_atlas_init(void);
extern void control_mixing_atlas_manual(void);

extern void control_mixing_atlas_attitude_enter(void);
extern void control_mixing_atlas_attitude(void);

extern void control_mixing_atlas_quad_enter(void);
extern void control_mixing_atlas_quad(void);

// extern void control_mixing_atlas_nav_enter(void);
// extern void control_mixing_atlas_nav_run(void);

#endif  // CONTROL_MIXING_ATLAS_H
