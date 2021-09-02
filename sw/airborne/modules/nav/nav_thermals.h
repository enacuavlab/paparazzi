/*
 * Copyright (C) Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
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

/** @file "modules/nav/nav_thermals.h"
 * @author Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
 * Thermals centering based on energy for gliders
 */

#ifndef NAV_THERMALS_H
#define NAV_THERMALS_H

#include "std.h"

struct NavThermalsParams {
  float roll;  ///< max velocity allowed
};

extern struct NavThermalsParams nav_thermals_params;

extern void nav_thermals_init(void);
extern void nav_thermals_setup(void);
extern float nav_thermals_run(void);
extern float nav_thermals_energy(float altitude, float speed);
extern float get_acc(void);
extern float average(float en[1000], int l);
extern bool exit_thermals(void);

#endif  // NAV_THERMALS_H
