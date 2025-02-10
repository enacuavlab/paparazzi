/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_poles_rotorcraft.c
 *
 * Turn around 2 points, with possible margins
 * Can be used in mission mode
 *
 */

#include "modules/nav/nav_poles_rotorcraft.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/waypoints.h"

uint8_t nav_poles_count = 0;

// local variables
static struct EnuCoor_f oval_wp1, oval_wp2;
static float oval_radius;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_poles_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 5) {
    uint8_t wp1_id = (uint8_t)(params[0]);
    uint8_t wp2_id = (uint8_t)(params[1]);
    float height = params[2];
    float radius = params[3];
    float margin = params[4];
    if (nav_poles_setup_wp(wp1_id, wp2_id, height, radius, margin)) {
      return nav_poles_run();
    } else {
      return false;
    }
  } else if (flag == MissionInit && nb == 7) {
    float height = params[4];
    struct LlaCoor_f lla1 = {
      .lat = RadOfDeg(params[0]),
      .lon = RadOfDeg(params[1]),
      .alt = stateGetLlaOrigin_f().alt + height
    };
    struct LlaCoor_f lla2 = {
      .lat = RadOfDeg(params[2]),
      .lon = RadOfDeg(params[3]),
      .alt = stateGetLlaOrigin_f().alt + height
    };
    float radius = params[5];
    float margin = params[6];
    if (nav_poles_setup_lla(&lla1, &lla2, height, radius, margin)) {
      return nav_poles_run();
    } else {
      return false;
    }
  } else if (flag == MissionRun) {
    return nav_poles_run();
  }
  return false; // not a valid case
}
#endif

void nav_poles_init(void)
{
  nav_poles_count = 0;
  oval_radius = 0.f;

#if USE_MISSION
  mission_register(nav_poles_mission, "POLES");
#endif
}

static bool compute_oval_points(struct EnuCoor_f *enu1, struct EnuCoor_f *enu2, float height, float radius, float margin)
{
  float dx = enu2->x - enu1->x;
  float dy = enu2->y - enu1->y;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 0.001f) {
    return false;
  }

  /* Unit vector from wp1 to wp2 */
  dx /= d;
  dy /= d;

  oval_wp1.x = enu1->x + (dx * (1.f - margin) - dy) * radius;
  oval_wp1.y = enu1->y + (dy * (1.f - margin) + dx) * radius;
  oval_wp1.z = height;

  oval_wp2.x = enu2->x - (dx * (1.f - margin) + dy) * radius;
  oval_wp2.y = enu2->y - (dy * (1.f - margin) - dx) * radius;
  oval_wp2.z = height;
  printf("%f %f |  %f %f | %f %f | %f %f\n", dx, dy, oval_wp1.x, oval_wp1.y, oval_wp2.x, oval_wp2.y, radius, margin);

  oval_radius = radius;
  NavVerticalAltitudeMode(height, 0.f);

  return true;
}

bool nav_poles_setup_wp(uint8_t wp1, uint8_t wp2, float height,
    float radius, float margin)
{
  struct EnuCoor_f *enu1 = waypoint_get_enu_f(wp1);
  struct EnuCoor_f *enu2 = waypoint_get_enu_f(wp2);
  if (enu1 == NULL || enu2 == NULL) {
    return false;
  }
  return compute_oval_points(enu1, enu2, height, radius, margin);
}

bool nav_poles_setup_lla(struct LlaCoor_f *lla1, struct LlaCoor_f *lla2, float height,
    float radius, float margin)
{
  struct EnuCoor_f enu1;
  struct EnuCoor_f enu2;
  enu_of_lla_point_f(&enu1, stateGetNedOrigin_f(), lla1);
  enu_of_lla_point_f(&enu2, stateGetNedOrigin_f(), lla2);
  return compute_oval_points(&enu1, &enu2, height, radius, margin);
}

bool nav_poles_run(void)
{
#ifdef NavOvalCount
  nav_poles_count = NavOvalCount;
#endif
  nav.nav_oval(&oval_wp2, &oval_wp1, oval_radius);
  return true; // TODO max number of laps ?
}


