/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/imav_rescue_people/imav_rescue_people.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Indentify people to be rescued for IMAV2026
 */

#ifndef IMAV_RESCUE_PEOPLE_H
#define IMAV_RESCUE_PEOPLE_H

#include "std.h"

extern void imav_rescue_init(void);

/** Update waypoint position to match the current best spot */
extern void imav_rescue_set_best_wp(uint8_t wp_id);


#endif  // IMAV_RESCUE_PEOPLE_H
