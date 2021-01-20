/*
 * Copyright (C) 2018-2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
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
 * @file modules/nav/nav_lace.c
 * @author VERDU Titouan
 *
 * Adaptive border pattern for cloud exploration
 */

#include "modules/nav/nav_lace.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"

enum LaceStatus {
  LACE_ENTER,
  LACE_INSIDE_START,
  LACE_INSIDE,
  LACE_OUTSIDE_START,
  LACE_OUTSIDE,
  LACE_RECOVER_START,
  LACE_RECOVER
};

enum RotationDir {
  LACE_LEFT,
  LACE_RIGHT
};

struct NavLace {
  enum LaceStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct EnuCoor_f last_border;
  struct EnuCoor_f estim_border;
  struct EnuCoor_f recover_circle;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
  float tps_in;
  float tps_out;
  float last_border_time;
  float recover_radius;
};

static struct NavLace nav_lace;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_lace.rotation == LACE_RIGHT) {
    rot_angle = -M_PI_2;
    nav_lace.rotation = LACE_LEFT;
  } else{
    rot_angle = M_PI_2;
    nav_lace.rotation = LACE_RIGHT;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_lace.radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_lace.radius);
  new_point.z = alt_sp;

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 8) {
    float start_x = params[0];
    float start_y = params[1];
    float start_z = params[2];
    int first_turn = params[3];
    float circle_radius = params[4];
    float vx = params[5];
    float vy = params[6];
    float vz = params[7];
    nav_lace_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_lace.pos_incr.x = vx * nav_dt;
    nav_lace.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_lace.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_lace_run();
  }
  return false; // not a valid case
}
#endif

// ABI message

#ifndef NAV_LACE_LWC_ID
#define NAV_LACE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_lace.inside_cloud = (bool) data[0];
  }
}

void nav_lace_init(void)
{
  nav_lace.status = LACE_ENTER;
  nav_lace.radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_LACE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_lace_mission, "LACE");
#endif
}

void nav_lace_setup(float init_x, float init_y,
                    float init_z, int turn,
                    float desired_radius, float vx,
                    float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_lace.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_lace.target = start;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
  nav_lace.radius = desired_radius;

  if (turn == 1) {
    nav_lace.rotation = LACE_RIGHT;
    nav_lace.radius_sign = 1.0f;
  } else {
    nav_lace.rotation = LACE_LEFT;
    nav_lace.radius_sign = -1.0f;
  }

  nav_lace.actual = *stateGetPositionEnu_f();
}

bool nav_lace_run(void)
{
  float pre_climb = 0.f;
  float max_recover_radius = 0.0;

  NavVerticalAutoThrottleMode(0.f); /* No pitch */

  switch (nav_lace.status) {
    case LACE_ENTER:
      // reach target point
      nav_route_xy(nav_lace.actual.x, nav_lace.actual.y, nav_lace.target.x, nav_lace.target.y);
      NavVerticalAltitudeMode(nav_lace.target.z + ground_alt, pre_climb);

      if (nav_lace.inside_cloud) {
        // found border or already inside
        nav_lace.status = LACE_INSIDE_START;
      }
      break;
    case LACE_INSIDE_START:
      // prepare inside circle
      nav_lace.actual = *stateGetPositionEnu_f();
      nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.target.z, nav_lace.direction);
      // reset circle counter
      nav_circle_radians = 0;
      nav_circle_radians_no_rewind = 0;
      // pepare recover
      nav_lace.last_border = nav_lace.actual;
      nav_lace.estim_border = nav_lace.actual;
      // fly inside
      nav_lace.status = LACE_INSIDE;
      break;
    case LACE_INSIDE:
      // increment center position
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      VECT3_ADD(nav_lace.estim_border, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      NavVerticalAltitudeMode(nav_lace.circle.z + ground_alt, pre_climb);

      if (!nav_lace.inside_cloud) {
        // found border, start outside
        nav_lace.status = LACE_OUTSIDE_START;
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      else if (NavCircleCountNoRewind() > MAX_CIRCLE_TURN) {
        // most likely lost inside
        nav_lace.status = LACE_RECOVER_START;
      }
      break;
    case LACE_OUTSIDE_START:
      // prepare outside circle
      nav_lace.actual = *stateGetPositionEnu_f();
      nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.circle.z, nav_lace.direction);
      // reset circle counter
      nav_circle_radians = 0;
      nav_circle_radians_no_rewind = 0;
      // prepare recover
      nav_lace.last_border = nav_lace.actual;
      nav_lace.estim_border = nav_lace.actual;
      // fly outside
      nav_lace.status = LACE_OUTSIDE;
      break;
    case LACE_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      VECT3_ADD(nav_lace.estim_border, nav_lace.pos_incr);
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      NavVerticalAltitudeMode(nav_lace.circle.z + ground_alt, pre_climb);

      if (nav_lace.inside_cloud) {
        // found border, start inside
        nav_lace.status = LACE_INSIDE_START;
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      else if (NavCircleCountNoRewind() > MAX_CIRCLE_TURN) {
        // most likely lost outside
        nav_lace.status = LACE_RECOVER_START;
        nav_lace.state_changed = true;
      }
      break;
    case LACE_RECOVER_START:
      // prepare recovery circle
      nav_lace.recover_circle.x = nav_lace.estim_border.x;
      nav_lace.recover_circle.y = nav_lace.estim_border.y;
      nav_lace.recover_circle.z = nav_lace.estim_border.z;
      // initial recovery radius
      nav_lace.recover_radius = nav_lace.radius;
      // FIXME
      max_recover_radius = sinf(Recover_angle) * sqrtf(powf(nav_lace.last_border.x - nav_lace.recover_circle.x, 2.0) + powf(nav_lace.last_border.y - nav_lace.recover_circle.y, 2.0));
      // reset circle counter
      nav_circle_radians = 0;
      nav_circle_radians_no_rewind = 0;
      break;
    case LACE_RECOVER:
      // increment center position
      VECT3_ADD(nav_lace.recover_circle, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.recover_circle.x, nav_lace.recover_circle.y , nav_lace.radius_sign * current_recover_radius);

      if (current_recover_radius < max_recover_radius) {
        current_recover_radius += 0.5;
      }

      if (nav_lace.inside_cloud) {
        nav_lace.status = LACE_INSIDE_START;
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      else {
        nav_lace.status = LACE_OUTSIDE_START;
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}
