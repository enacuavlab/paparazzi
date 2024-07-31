/*
 * Copyright (C) 
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_udwadia.c
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_udwadia.h"


void stabilization_attitude_enter(void)
{
  stabilization_udwadia_enter();
}

void stabilization_attitude_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_udwadia_run(in_flight, sp, thrust, cmd);
}

