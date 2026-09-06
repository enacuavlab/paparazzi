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

/** @file "modules/imav_rescue_people/scoutector.h"
 * @author Fabien-B <fabien-B@github.com>
 * Scoutector driver, specially made for IMAV 2026 !
 */

#ifndef SCOUTECTOR_H
#define SCOUTECTOR_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

typedef struct {
  float det;
  float snr;
  float lit;
} scoutector_t;

extern void scoutector_init(void);
extern void scoutector_report(void);

extern scoutector_t scout_data;

#ifndef SCOUT_MAP_SIZE
#define SCOUT_MAP_SIZE 51
#endif

/** map of scout detection
 */
struct scout_map_t {
  scoutector_t grid[SCOUT_MAP_SIZE][SCOUT_MAP_SIZE];
  struct NedCoor_f center;                            ///< center of the map
  float res;                                          ///< map resolution in m/cell
};

extern struct scout_map_t scout_map;

/** init map
 * @param[in] map pointer to map
 * @param[in] pos position of the map center
 * @param[in] res resolution of the map in m per cell
 */
extern void scout_map_init(struct scout_map_t *map, struct NedCoor_f pos, float res);

/** update map with new data
 * @param[in] map pointer to map
 * @param[in] data new data
 * @param[in] pos position of the new data
 * @return false is position is outside the map
 */
extern bool scout_mat_update(struct scout_map_t *map, scoutector_t data, struct NedCoor_f pos);

/** get weighted center of the measurements (snr), expected to be the scout position
 * @param[in] map pointer to map
 * @param[out] pos pointer to computed position
 * @return signal quality (the averaged snr level FIXME really needed ?)
 */
extern float scout_mat_get_barycenter(struct scout_map_t *map, struct NedCoor_f *pos);

/** update waypoint position from map
 * @param[in] map pointer to map
 * @param[in] wp_id waypoint ID
 */
extern void scout_map_update_wp(struct scout_map_t *map, uint8_t wp_id);

#endif  // SCOUTECTOR_H
