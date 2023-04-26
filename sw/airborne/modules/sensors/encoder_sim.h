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

/** @file "modules/sensors/encoder_sim.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Driver for encoder simulation.
 * add <module name="encoder" type="sim"/>
 */

#ifndef ENCODER_SIM_H
#define ENCODER_SIM_H

#include "filters/high_gain_filter.h"
#include "peripherals/amt22.h"

struct EncoderAmt22 {
  struct amt22_t amt22;
  struct high_gain_filter H_g_filter;
};

extern struct EncoderAmt22 encoder_amt22;

extern void encoder_sim_init(void);
extern void encoder_sim_periodic(void);
extern void encoder_sim_upadate(float angle, float ag_speed, float ag_accel);


#endif  // ENCODER_SIM_H
