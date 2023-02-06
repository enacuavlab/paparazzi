/*
 * Copyright (C) 2023 Flo&Fab <name.surname@enac.fr>
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

/** @file "modules/sensors/encoder_amt22.c"
 * @author Flo&Fab <name.surname@enac.fr>
 * Driver for AMT22 encoder from CUI devices.
 */

#include "modules/sensors/encoder_amt22.h"
#include "peripherals/amt22.h"

#include "filters/high_gain_filter.h"

#include "modules/datalink/downlink.h"


struct amt22_t amt22;

struct high_gain_filter H_g_filter_rot;

void encoder_amt22_init(void)
{
  amt22_init(&amt22, &AMT22_SPI_DEV, AMT22_SPI_SLAVE_IDX);
  high_gain_filter_init(&H_g_filter_rot, 1, 1.3, 0.06, 500, 0, 0);
}

void encoder_amt22_periodic(void)
{
  //amt22_request(&amt22, AMT22_READ_TURNS);
  amt22_request(&amt22, AMT22_READ_POSITION);
  high_gain_filter_process(&H_g_filter_rot, amt22.angle_rad);
  float f[4] = {amt22.position, amt22.angle_rad, H_g_filter_rot.hatx[0], H_g_filter_rot.hatx[1]};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, f);

}

void encoder_amt22_event(void)
{
  amt22_event(&amt22);
}


