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

#include "modules/datalink/downlink.h"

struct amt22_t amt22;

void encoder_amt22_init(void)
{
  amt22_init(&amt22, &AMT22_SPI_DEV, AMT22_SPI_SLAVE_IDX);
}

void encoder_amt22_periodic(void)
{
  amt22_request(&amt22, AMT22_READ_TURNS);

  float f[2] = {amt22.position, amt22.turns};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, f);

}

void encoder_amt22_event(void)
{
  amt22_event(&amt22);
}


