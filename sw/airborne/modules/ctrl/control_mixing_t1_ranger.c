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

/** @file "modules/ctrl/control_mixing_t1_ranger.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing specific to the Heewing T1 Ranger
 */

#include "modules/ctrl/control_mixing_t1_ranger.h"
#include "modules/radio_control/radio_control.h"
#include "generated/radio.h"
#include "modules/core/commands.h"

// Tilt position in forward flight
#ifndef CMTR_TILT_FORWARD
#define CMTR_TILT_FORWARD 0
#endif

// Tilt vertical position for hovering
#ifndef CMTR_TILT_VERTICAL
#define CMTR_TILT_VERTICAL 7400
#endif

void control_mixing_t1_ranger_init(void)
{
  // your init code here
}

void control_mixing_t1_ranger_manual(void)
{
  commands[COMMAND_ROLL] = radio_control_get(RADIO_ROLL);
  commands[COMMAND_PITCH] = radio_control_get(RADIO_PITCH);
  commands[COMMAND_YAW] = 0;
  commands[COMMAND_TILT] = CMTR_TILT_FORWARD;
  commands[COMMAND_MOTOR_RIGHT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_LEFT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_TAIL] = MIN_PPRZ;
}

void control_mixing_t1_ranger_hover(void)
{
}

