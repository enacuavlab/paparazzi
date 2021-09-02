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

/** @file "modules/nav/nav_thermal_circle.h"
 * @author Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
 * Thermals centering based on circles for gliders
 */

#ifndef NAV_THERMAL_CIRCLE_H
#define NAV_THERMAL_CIRCLE_H
#define PI 3.14159

#include "std.h"

extern void nav_thermal_circle_init(void);
extern bool circle(void);
extern void thermal_circle_setup(char id);


#endif  // NAV_THERMAL_CIRCLE_H
