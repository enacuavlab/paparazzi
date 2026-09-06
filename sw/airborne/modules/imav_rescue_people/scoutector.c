/*
 * Copyright (C) 2026 Fabien-B <fabien-B@github.com>
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

/** @file "modules/imav_rescue_people/scoutector.c"
 * @author Fabien-B <fabien-B@github.com>
 * Scoutector driver, specially made for IMAV 2026 !
 */

#include "modules/imav_rescue_people/scoutector.h"
#include "string.h"
#include "uavcan/uavcan.h"
#include "uavcan.protocol.debug.KeyValue.h"
#include "modules/datalink/downlink.h"
#include "modules/nav/waypoints.h"
#include <generated/flight_plan.h>

// FIXME make a better config
#ifndef WP_SEARCH
#define WP_SEARCH 0
#endif

static uavcan_event scoutector_uavcan_ev;

scoutector_t scout_data;

struct scout_map_t scout_map;

static void scoutector_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  struct uavcan_protocol_debug_KeyValue msg;

  if(uavcan_protocol_debug_KeyValue_decode(transfer, &msg)) {
    return;
  }

  if(msg.key.len != 3) {
    return;
  }

  if(strncmp("det", (const char*)msg.key.data, 3) == 0) {
    scout_data.det = msg.value;
  } else if(strncmp("snr", (const char*)msg.key.data, 3) == 0) {
    scout_data.snr = msg.value;
  } else if(strncmp("lit", (const char*)msg.key.data, 3) == 0) {
    scout_data.lit = msg.value;
  }
  //debug
  else if(strncmp("lct", (const char*)msg.key.data, 3) == 0) {
    //TODO log ?
  } else if(strncmp("ltu", (const char*)msg.key.data, 3) == 0) {
    //TODO log ?
  }

  // update map
  scout_mat_update(&scout_map, scout_data, *stateGetPositionNed_f());
}


void scoutector_init(void)
{
  struct EnuCoor_f enu = *waypoint_get_enu_f(WP_SEARCH);
  struct NedCoor_f ned;
  ENU_OF_TO_NED(ned, enu);
  scout_map_init(&scout_map, ned, 1.f);

  uavcan_bind(UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID, UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE, &scoutector_uavcan_ev, &scoutector_uavcan_cb);
}

void scoutector_report(void) {
  float f[3] = {scout_data.det, scout_data.snr, scout_data.lit};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3, f);
}


/*******
 * MAP *
 *******/

void scout_map_init(struct scout_map_t *map, struct NedCoor_f pos, float res)
{
  map->res = res;
  map->center = pos;
  for (int i = 0; i < SCOUT_MAP_SIZE; i++) {
    for (int j = 0; j < SCOUT_MAP_SIZE; j++) {
      map->grid[i][j].det = 0.f;
      map->grid[i][j].snr = 0.f;
      map->grid[i][j].lit = 0.f;
    }
  }
}

bool scout_mat_update(struct scout_map_t *map, scoutector_t data, struct NedCoor_f pos)
{
  int x = (int)((pos.x - map->center.x) / map->res + SCOUT_MAP_SIZE / 2.f + 0.5f);
  int y = (int)((pos.y - map->center.y) / map->res + SCOUT_MAP_SIZE / 2.f + 0.5f);
  if (x < 0 || x >= SCOUT_MAP_SIZE || y < 0 || y >= SCOUT_MAP_SIZE) {
    return false; // out of map
  }
  map->grid[x][y] = data; // TODO filter with current data in cell
  return true;
}

float scout_mat_get_barycenter(struct scout_map_t *map, struct NedCoor_f *pos)
{
  float bx = 0.f;
  float by = 0.f;
  float quality = 0.f;
  int nb_qual = 0;
  float sum = 0.f;
  const int offset = (int)(SCOUT_MAP_SIZE / 2.f + 0.5f);
  for (int i = 0; i < SCOUT_MAP_SIZE; i++) {
    for (int j = 0; j < SCOUT_MAP_SIZE; j++) {
      bx += map->grid[i][j].snr * (float)(i - offset);
      by += map->grid[i][j].snr * (float)(j - offset);
      sum += map->grid[i][j].snr;
      if (map->grid[i][j].snr > 0.f) {
        quality += map->grid[i][j].snr;
        nb_qual++;
      }
    }
  }
  pos->x = map->center.x + (bx * map->res / sum);
  pos->y = map->center.y + (by * map->res / sum);
  pos->z = map->center.z;
  quality /= (float)nb_qual;
  return quality;
}

void scout_map_update_wp(struct scout_map_t *map, uint8_t wp_id)
{
  struct NedCoor_f pos;
  if (scout_mat_get_barycenter(map, &pos) > 0.f) {
    struct EnuCoor_f enu;
    ENU_OF_TO_NED(enu, pos);
    waypoint_set_enu(wp_id, &enu);
  }
}

