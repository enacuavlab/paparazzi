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
#include <stdio.h>

float thermal_ctl = 0;
float thermal_threshold = 50;

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

void thermal_ctl_thermal_threshold_handler(float value) {
  thermal_threshold = value;
  uint8_t tab[2] = {'t', (uint8_t)value};
  uint8_t dst_id = 0;
  DOWNLINK_SEND_PAYLOAD_COMMAND(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &dst_id, 2, tab);
}

void thermal_clusters_cb(uint8_t* buf)
{
  // PAYLOAD_FLOAT: cluster_key,east,north,up,latitude,longitude,median_temperature,member_count

  int nb = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
  if(nb != 8) {return;}

  float *b = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);
  uint8_t cluster_idx = b[0];
  // struct EnuCoor_f enu = {.x = b[1], .y = b[2], .z = b[3]};
  float lat = b[4];
  float lon = b[5];
  float temp = b[6];
  int count = b[7];

  uint8_t color = 0 << 6 | 5 << 3 | 5;  // Transparent, Magenta, Magenta
  uint8_t shape = 0;
  uint8_t status = 0;
  float radius = 2;
  int32_t latarr[] = {lat * 1e7};
  int32_t lonarr[] = {lon * 1e7};
  char text[10];
  int nb_text = snprintf(text, 10, "%.0fC (%d)", temp, count);

  DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice, &cluster_idx, &color, &shape, &status, &radius, 1, latarr, 1, lonarr, nb_text, text);

}


