/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
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

/** @file "modules/digital_cam/thermal_ctl.c"
 * @author Fabien-B <fabien-b@github.com>
 * Control payload via PAYLOAD_COMMAND messages.
 */

#include "modules/digital_cam/thermal_ctl.h"
#include "modules/datalink/downlink.h"
#include "modules/datalink/extra_pprz_dl.h"

float thermal_ctl = 0;

#define THERMAL_CTL_NONE    0
#define THERMAL_CTL_PWR_OFF 1
#define THERMAL_CTL_REC_ON  2
#define THERMAL_CTL_REC_OFF 3


void thermal_ctl_thermal_ctl_handler(float value) {
  int val = (int)value;
  uint8_t tab[2];
  uint8_t dst_id = 0;
  if(val == THERMAL_CTL_PWR_OFF) {
    tab[0] = 's';
    tab[1] = 0;
  } else if(val == THERMAL_CTL_REC_ON) {
    tab[0] = 'r';
    tab[1] = 1;
  } else if(val == THERMAL_CTL_REC_OFF) {
    tab[0] = 'r';
    tab[1] = 0;
  } else {
    return;
  }

  DOWNLINK_SEND_PAYLOAD_COMMAND(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &dst_id, 2, tab);

}

void thermal_status_cb(uint8_t* buf)
{
  // PAYLOAD_COMMAND
  // your datalink code here
}


