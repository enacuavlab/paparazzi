/*
 * Copyright (C) Jacson Barth <jacsonm2@gmail.com>
 * ENAC UAV Lab
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

/** @file stabilization_attitude_quat_mfc.c
 * @brief ENAC UAV Lab
 * This control algorithm is Model-Free Control (MFC)
 *
 * This is an implementation of the publication in the
 * International American Control Conference : Full Model-Free 
 * Control Architecture for Hybrid UAVs
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"


void stabilization_attitude_init(void)
{
  // Check if the indi init is already done for rate control
  //#ifndef STABILIZATION_RATE_MFC
  //  stabilization_mfc_init();
  //#endif
}

void stabilization_attitude_enter(void)
{
  stabilization_mfc_enter();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  stabilization_mfc_set_failsafe_setpoint();
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  stabilization_mfc_set_rpy_setpoint_i(rpy);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  stabilization_mfc_set_earth_cmd_i(cmd, heading);
}

void stabilization_attitude_run(bool in_flight)
{
  stabilization_mfc_run(in_flight);
}

void stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  stabilization_mfc_read_rc(in_flight, in_carefree, coordinated_turn);
}
