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
  float kp;
  float kd;
};

extern struct BodyStab body_stab;

extern void body_stabilisation_init(void);
extern void body_stabilisation_periodic(void);

/**
 * settings handlers
 */
extern void body_stabilisation_reset(float enabled);
extern void body_stabilisation_update_kp(float kp);
extern void body_stabilisation_update_kd(float kd);

#endif  // BODY_STABILISATION_H
