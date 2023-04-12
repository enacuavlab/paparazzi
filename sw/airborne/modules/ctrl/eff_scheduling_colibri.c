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

/** @file "modules/ctrl/eff_scheduling_colibri.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Interpolation of control effectivenss matrix of the Colibri.
 */

#include "modules/ctrl/eff_scheduling_colibri.h"

// Airspeed at which only with motor
#ifndef INDI_SCHEDULING_LOW_AIRSPEED
#define INDI_SCHEDULING_LOW_AIRSPEED 8.0
#endif

#if INDI_NUM_ACT < 8
#error "This module works with Colibri with 8 actuators by only 6 are managed by INDI"
#endif


void ctrl_eff_scheduling_init(void)
{
  // your init code here
}

void ctrl_eff_scheduling_periodic(void)
{
  // your periodic code here.
  // freq = 20.0 Hz
}


