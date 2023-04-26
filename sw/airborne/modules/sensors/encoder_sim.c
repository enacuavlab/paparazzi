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

/** @file "modules/sensors/encoder_sim.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Driver for encoder simulation.
 */

#include "modules/sensors/encoder_sim.h"


struct EncoderAmt22 encoder_amt22;

void encoder_sim_init(void)
{

}

void encoder_sim_periodic(void)
{

}

void encoder_sim_upadate(float angle, float ag_speed, float ag_accel){
  encoder_amt22.H_g_filter.hatx[0] = angle;
  encoder_amt22.H_g_filter.hatx[1] = ag_speed;
  encoder_amt22.H_g_filter.hatx[2] = ag_accel;
}





