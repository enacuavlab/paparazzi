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

#define SCOUT_MAP_DEBUG 0

#if (defined SITL) && SCOUT_MAP_DEBUG
#define DEBUG_PRINT printf
#else
#define DEBUG_PRINT(...) {}
#endif

#ifndef SCOUT_MAP_THRESHOLD
#define SCOUT_MAP_THRESHOLD 0.8f
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

void scoutector_sim(void) {
#if (defined SITL) && (defined WP_SCOUT)
  struct EnuCoor_f scout = *waypoint_get_enu_f(WP_SCOUT);
  struct EnuCoor_f pos = *stateGetPositionEnu_f();
  struct FloatVect3 dp;
  VECT3_DIFF(dp, pos, scout);
  float dist = float_vect3_norm(&dp);
  // scout at 3 m ~ 95 dB
  // sound level: L(d) = L(d0) - 20 * log10(d/d0)
  // snr = 20 db at 3 m with propellers -> L(d0) = 20 (FIXME check correct value)
  float snr = 20. - 20.f*log10f(dist/3.f);
  if (snr > 3.f) { //FIXME check with Alex
    scout_data.snr = snr;
  } else {
    scout_data.snr = 0.f;
  }
  scout_mat_update(&scout_map, scout_data, *stateGetPositionNed_f());
#endif
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

void scout_map_reset(struct scout_map_t *map)
{
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
  float max = 0.f;
  //float mx = 0.f;
  //float my = 0.f;
  for (int i = 0; i < SCOUT_MAP_SIZE; i++) {
    for (int j = 0; j < SCOUT_MAP_SIZE; j++) {
      if (map->grid[i][j].snr > max) {
        max = map->grid[i][j].snr;
        // mx = (float)(i - offset) * map->res;
        // my = (float)(j - offset) * map->res;
      }
    }
  }
  if (max < 1e-5) {
    *pos = map->center;
    return 0.f; // nothing in the map
  }
  DEBUG_PRINT("map:\n");
  for (int i = 0; i < SCOUT_MAP_SIZE; i++) {
    for (int j = 0; j < SCOUT_MAP_SIZE; j++) {
      float val = powf(map->grid[i][j].snr/max, 2.f);
      //float val = map->grid[i][j].snr/max;
      DEBUG_PRINT("\t%.1f,", val);
      if (val > SCOUT_MAP_THRESHOLD) {
        bx += val * (float)(i - offset);
        by += val * (float)(j - offset);
        sum += val;
        quality += map->grid[i][j].snr;
        nb_qual++;
      }
    }
    DEBUG_PRINT("\n");
  }
  pos->x = map->center.x + (bx * map->res / sum);
  pos->y = map->center.y + (by * map->res / sum);
  pos->z = map->center.z;
  quality /= (float)nb_qual;
  DEBUG_PRINT(" -> offset %d, sum %f, res %f, bx %f, by %f, cx %f, cy %f, mx %f, my %f\n", offset, sum, map->res, bx, by, map->center.x, map->center.y, mx, my);
  DEBUG_PRINT(" -> pos %.4f %.4f, quality=%.2f\n", pos->x, pos->y, quality);
#ifdef WP_SCOUT
  DEBUG_PRINT(" -> scout %f %f\n", waypoint_get_enu_f(WP_SCOUT)->y, waypoint_get_enu_f(WP_SCOUT)->x); // ENU -> NED
#endif
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

