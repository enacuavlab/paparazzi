/*
 * Copyright (C) 2020 Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/nav/navigation_fish.c"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Bio-inspired swarm navigation
 *
 * Based on:
 * Calovi DS, Litchinko A, Lecheval V, Lopez
 * U, Pérez Escudero A, Chaté H, et al. (2018)
 * Disentangling and modeling interactions in fish
 * with burst-and-coast swimming reveal distinct
 * alignment and attraction behaviors. PLoS Comput
 * Biol 14(1): e1005933.
 * https://doi.org/10.1371/journal.pcbi.1005933
 */

#include "modules/nav/swarm_fish.h"
#include "modules/multi/traffic_info.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"
#include "autopilot.h"
#include "autopilot_guided.h"
#include "modules/nav/waypoints.h"
#include "math/pprz_geodetic_float.h"
#include "generated/flight_plan.h"
#include <math.h>
#include "modules/datalink/telemetry.h"

#include <stdio.h>
#include <stdlib.h>

/**
 * Default parameters
 */

#ifndef SWARM_FISH_MAX_SPEED
#define SWARM_FISH_MAX_SPEED 5.0f
#endif

#ifndef SWARM_FISH_MIN_SPEED
#define SWARM_FISH_MIN_SPEED 0.5f
#endif

#ifndef SWARM_FISH_MAX_YAW
#define SWARM_FISH_MAX_YAW RadOfDeg(90.f)
#endif

#ifndef SWARM_FISH_MAX_ALT
#define SWARM_FISH_MAX_ALT 50.0f
#endif

#ifndef SWARM_FISH_MIN_ALT
#define SWARM_FISH_MIN_ALT 2.0f
#endif

#ifndef SWARM_FISH_ARENA_RADIUS
#define SWARM_FISH_ARENA_RADIUS 15.f
#endif

#ifndef SWARM_FISH_USE_HEADING
#define SWARM_FISH_USE_HEADING true
#endif

#ifndef SWARM_FISH_FREE_HEADING
#define SWARM_FISH_FREE_HEADING true
#endif

#ifndef SWARM_FISH_VZ_CTL
#define SWARM_FISH_VZ_CTL false
#endif

#ifndef SWARM_FISH_Y_ATT
#define SWARM_FISH_Y_ATT 1.0f
#endif

#ifndef SWARM_FISH_L_ATT
#define SWARM_FISH_L_ATT 6.0f
#endif

#ifndef SWARM_FISH_D_ATT
#define SWARM_FISH_D_ATT 2.f
#endif

#ifndef SWARM_FISH_A_ATT
#define SWARM_FISH_A_ATT 0.3f
#endif

#ifndef SWARM_FISH_Y_ALI
#define SWARM_FISH_Y_ALI 0.5f
#endif

#ifndef SWARM_FISH_L_ALI
#define SWARM_FISH_L_ALI 3.f
#endif

#ifndef SWARM_FISH_A_ALI
#define SWARM_FISH_A_ALI 0.0f
#endif

#ifndef SWARM_FISH_Y_V
#define SWARM_FISH_Y_V 0.5f
#endif

#ifndef SWARM_FISH_L_V
#define SWARM_FISH_L_V 3.f
#endif

#ifndef SWARM_FISH_A_V
#define SWARM_FISH_A_V 0.0f
#endif

#ifndef SWARM_FISH_Y_Z
#define SWARM_FISH_Y_Z 0.3f
#endif

#ifndef SWARM_FISH_L_Z
#define SWARM_FISH_L_Z 4.f
#endif

#ifndef SWARM_FISH_D_Z
#define SWARM_FISH_D_Z 2.0f
#endif

#ifndef SWARM_FISH_A_Z
#define SWARM_FISH_A_Z 0.0f
#endif

#ifndef SWARM_FISH_Y_NAV
#define SWARM_FISH_Y_NAV 0.3f
#endif

#ifndef SWARM_FISH_Y_Z_NAV
#define SWARM_FISH_Y_Z_NAV 0.5f
#endif

#ifndef SWARM_FISH_L_Z_NAV
#define SWARM_FISH_L_Z_NAV 1.f
#endif

#ifndef SWARM_FISH_Y_IN
#define SWARM_FISH_Y_IN 1.0f
#endif

#ifndef SWARM_FISH_L_IN
#define SWARM_FISH_L_IN 4.0f
#endif

#ifndef SWARM_FISH_A_IN
#define SWARM_FISH_A_IN 0.2f
#endif

#ifndef SWARM_FISH_Y_Z_IN
#define SWARM_FISH_Y_Z_IN 1.0f
#endif

#ifndef SWARM_FISH_L_Z_IN
#define SWARM_FISH_L_Z_IN 4.0f
#endif

#ifndef SWARM_FISH_D_Z_IN
#define SWARM_FISH_D_Z_IN 1.0f
#endif

#ifndef SWARM_FISH_Y_OBS
#define SWARM_FISH_Y_OBS 1.0f
#endif

#ifndef SWARM_FISH_Y_V_OBS
#define SWARM_FISH_Y_V_OBS 1.0f
#endif

#ifndef SWARM_FISH_L_OBS
#define SWARM_FISH_L_OBS 4.0f
#endif

#ifndef SWARM_FISH_A_OBS
#define SWARM_FISH_A_OBS 0.5f
#endif

#ifndef SWARM_FISH_V0_OBS
#define SWARM_FISH_V0_OBS 1.0f
#endif

#ifndef SWARM_FISH_Y_Z_OBS
#define SWARM_FISH_Y_Z_OBS 0.5f
#endif

#ifndef SWARM_FISH_L_Z_OBS
#define SWARM_FISH_L_Z_OBS 0.2f
#endif


struct SwarmFishParams swarm_fish_params;
// shortname to params
#define sfp swarm_fish_params

struct SwarmFishNav swarm_fish_nav;
// shortname to nav params
#define sfn swarm_fish_nav

//state data
struct SwarmFish {
  float heading;        ///< heading command
  float theta_obs;      ///< angle to wall
  float f_social[3];    ///< social interactions
  float f_nav[3];       ///< navigation
  float f_intruder[3];  ///< intruder repulsion
  float f_obstacle[3];  ///< obstacle repulsion
};

static struct SwarmFish swarm_fish;

/** initialization of parameters and state variables
*/
void swarm_fish_init(void)
{
  swarm_fish_params.y_att        = SWARM_FISH_Y_ATT;
  swarm_fish_params.l_att        = SWARM_FISH_L_ATT;
  swarm_fish_params.d_att        = SWARM_FISH_D_ATT;
  swarm_fish_params.a_att        = SWARM_FISH_A_ATT;
  swarm_fish_params.y_ali        = SWARM_FISH_Y_ALI;
  swarm_fish_params.l_ali        = SWARM_FISH_L_ALI;
  swarm_fish_params.a_ali        = SWARM_FISH_A_ALI;
  swarm_fish_params.y_v          = SWARM_FISH_Y_V;
  swarm_fish_params.l_v          = SWARM_FISH_L_V;
  swarm_fish_params.a_v          = SWARM_FISH_A_V;
  swarm_fish_params.y_z          = SWARM_FISH_Y_Z;
  swarm_fish_params.l_z          = SWARM_FISH_L_Z;
  swarm_fish_params.d_z          = SWARM_FISH_D_Z;
  swarm_fish_params.a_z          = SWARM_FISH_A_Z;
  swarm_fish_params.y_nav        = SWARM_FISH_Y_NAV;
  swarm_fish_params.y_z_nav      = SWARM_FISH_Y_Z_NAV;
  swarm_fish_params.l_z_nav      = SWARM_FISH_L_Z_NAV;
  swarm_fish_params.y_in         = SWARM_FISH_Y_IN;
  swarm_fish_params.l_in         = SWARM_FISH_L_IN;
  swarm_fish_params.a_in         = SWARM_FISH_A_IN;
  swarm_fish_params.y_z_in       = SWARM_FISH_Y_Z_IN;
  swarm_fish_params.l_z_in       = SWARM_FISH_L_Z_IN;
  swarm_fish_params.a_z_in       = SWARM_FISH_A_Z_IN;
  swarm_fish_params.y_obs        = SWARM_FISH_Y_OBS;
  swarm_fish_params.y_v_obs      = SWARM_FISH_Y_V_OBS;
  swarm_fish_params.l_obs        = SWARM_FISH_L_OBS;
  swarm_fish_params.a_obs        = SWARM_FISH_A_OBS;
  swarm_fish_params.v0_obs       = SWARM_FISH_V0_OBS;
  swarm_fish_params.y_z_obs      = SWARM_FISH_Y_Z_OBS;
  swarm_fish_params.l_z_obs      = SWARM_FISH_L_Z_OBS;

  swarm_fish_params.max_speed    = SWARM_FISH_MAX_SPEED;
  swarm_fish_params.min_speed    = SWARM_FISH_MIN_SPEED;
  swarm_fish_params.max_yaw      = SWARM_FISH_MAX_YAW;
  swarm_fish_params.max_alt      = SWARM_FISH_MAX_ALT;
  swarm_fish_params.min_alt      = SWARM_FISH_MIN_ALT;
  swarm_fish_params.arena_radius = SWARM_FISH_ARENA_RADIUS;
  swarm_fish_params.use_heading  = SWARM_FISH_USE_HEADING;
  swarm_fish_params.free_heading = SWARM_FISH_FREE_HEADING;
  swarm_fish_params.vz_ctl       = SWARM_FISH_VZ_CTL;
}


static inline void send_swarm_message(void)
{
  // TODO update message fields
  //DOWNLINK_SEND_SWARM_FISH(DefaultChannel, DefaultDevice,
  //    &swarm_fish.heading,
  //    &swarm_fish.step_size,
  //    &swarm_fish.r_w,
  //    &swarm_fish.f_w,
  //    &swarm_fish.theta_w,
  //    &swarm_fish.f_fluct,
  //    &swarm_fish.f_wall,
  //    &swarm_fish.f_ali,
  //    &swarm_fish.f_att);
}

/** sign function
 *
 * @param number x
 * @return sign of x
 */
static float sign(float x)
{
  if(x >= 0.f) {
    return 1.f;
  } else {
    return -1.f;
  }
}

/** anisotropic function
 *
 * @param a anisotropy coef (0 means isotropic, gain = 1.)
 * @param psi angle of perseption
 * @return anisotripic gain in [-1., 1.]
 */
static float anisotripic(float a, float psi)
{
  return (1.f + a * cosf(psi)) / (1.f + a);
}

/** Calculates distance between the uav and the arena wall
 *  centered on a WP
 *
 * @param wp_id waypoint ID
 * @param pos ENU coordinates
 * @return distance to circular wall, negatif if outside
 */
static float distance_to_arena(uint8_t wp_id, struct EnuCoor_f *pos)
{
  float x_center = waypoint_get_x(wp_id);
  float y_center = waypoint_get_y(wp_id);
  float dist = sfp.arena_radius - sqrtf(((pos->x - x_center) * (pos->x - x_center)) + ((pos->y - y_center) * (pos->y - y_center)));
  return dist;
}

/** calculates the relative orientation to the wall
 *
 * @param pos ENU coordinates
 * @param psi angle from north clockwise
 * @return angle to circular wall
 */
static float angle_to_wall(struct EnuCoor_f *pos, float psi)
{
  float dir = M_PI_2 - psi;
  float theta = atan2f(pos->y, pos->x);
  float delta = dir - theta;
  FLOAT_ANGLE_NORMALIZE(delta);
  return delta;
}

/** calculates the horizontal distance between two uavs
 *
 * @param pos ENU coordinates of the first drone
 * @param other ENU coordinates of the second drone
 * @return the distance between two uavs from their given position
 */
static float get_distance_2D(struct EnuCoor_f *pos, struct EnuCoor_f *other)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  return FLOAT_VECT2_NORM(diff);
}

/** calculates the 3D distance between two uavs
 *
 * @param pos ENU coordinates of the first drone
 * @param other ENU coordinates of the second drone
 * @return the distance between two uavs from their given position
 */
static float get_distance_3D(struct EnuCoor_f *pos, struct EnuCoor_f *other)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  return FLOAT_VECT3_NORM(diff);
}

/** calculates a uav's viewing angle on another uav
 * @param pos ENU coordinates of the viewer drone
 * @param other ENU coordinates of the viewed drone
 * @param psi heading of the viewer drone
 * @return viewing angle
 */
static float viewing_angle(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  float dir = 0.f;
  if (fabsf(diff.x) < 1e-5f) {
    dir = sign(diff.y) * M_PI_2;
  } else {
    dir = atanf(diff.y/diff.x);
  }
  return (sign(diff.x)*M_PI_2) - psi - dir;
}

/** calculates difference between two headings
 * @param psi first heading
 * @param psi_other second heading
 * @return the difference between the two headings
 */
static float delta_phi(float psi, float psi_other)
{
  float dp = psi_other - psi;
  FLOAT_ANGLE_NORMALIZE(dp);
  return dp;
}

/** calculates the influence of a uav on a neighbor of his
 * @param pos ENU coordinates of the influenced uav
 * @param other ENU coordinates of the influencing uav
 * @param psi heading of the influenced uav
 * @param psi_other heading of the influencing uav
 * @return influence
 */
static float neighbor_influence(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi , float psi_other)
{
  float view = viewing_angle(pos, other, psi);
  float d2d = distance_drone_to_drone(pos, other);
  float d_phi = delta_phi(psi, psi_other);
  float tmp_ali = d2d / sfp.l_ali;
  tmp_ali = sfp.y_ali * sinf(d_phi) * ((d2d + sfp.d0_ali) / sfp.l_ali) * expf(-tmp_ali * tmp_ali);
  float amplifier = 1.f;
  if (d2d < sfp.d0_att) {
    amplifier = expf(sfp.alpha_rep * ((sfp.d0_att - d2d)/(sfp.d0_att)));
  }
  float tmp_att = d2d / sfp.l_att;
  tmp_att = sfp.y_att * (((d2d - sfp.d0_att) / sfp.l_att) / (1.f + tmp_att * tmp_att)) * sinf(view) * amplifier;
  return fabs(tmp_att + tmp_ali);
}

/** calculates the attraction effect between two uavs
 * @param pos ENU coordinates of the current uav
 * @param other ENU coordinates of the neighbor uav
 * @param psi heading of the current uav
 * @return attraction
 */
static float neighbor_attraction(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi)
{
  float view = viewing_angle(pos, other, psi);
  float d2d = distance_drone_to_drone(pos, other);

  if(d2d <= sfp.min_d2d) {
    if (view > -M_PI_2 && view <= M_PI_2) {
      swarm_fish.velocity = swarm_fish.velocity - sfp.min_velocity;
    } else {
      swarm_fish.velocity = swarm_fish.velocity + sfp.min_velocity;
    }
  }
  float amplifier = 1.f;
  if (d2d < sfp.d0_att) {
    amplifier = expf(sfp.alpha_rep * ((sfp.d0_att - d2d) / sfp.d0_att));
  }
  float tmp_att = d2d / sfp.l_att;
  tmp_att = sfp.y_att * (((d2d - sfp.d0_att) / sfp.l_att) / (1.f + tmp_att * tmp_att)) * sinf(view) * amplifier;
  return tmp_att;
}

/** calculates the alignement effect between two uavs
 * @param pos ENU coordinates of the current uav
 * @param other ENU coordinates of the neighbor uav
 * @param psi heading of the current uav
 * @param psi_other heading of the neighbor uav
 * @return alignement
 */
static float neighbor_alignement(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi , float psi_other)
{
  float d2d = distance_drone_to_drone(pos, other);
  float d_phi = delta_phi(psi, psi_other);
  float tmp_ali = d2d / sfp.l_ali;
  tmp_ali = sfp.y_ali * sinf(d_phi) * ((d2d + sfp.d0_ali) / sfp.l_ali) * expf(-tmp_ali * tmp_ali);
  return tmp_ali;
}

//static float perpendicular_distance(float x, float y, float sx, float sy, float dx, float dy)
//{
//  float agent = (dy * (y - sy) + dx * (x - sx)) / (dx * dx + dy * dy);
//  float x_new = sx + dx * agent;
//  float y_new = sy + dy * agent;
//  return sqrtf((x - x_new) * (x - x_new) + (y - y_new) * (y - y_new));
//}

/** calculates new variation of the heading for the uav based on current state
 * @return heading variation
 */
static float calculate_new_heading(void)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float psi = stateGetNedToBodyEulers_f()->psi;

  // compute distance and angle to wall
  swarm_fish.r_w = distance_to_wall(pos);
  swarm_fish.theta_w = angle_to_wall(pos, psi);

  // trajectory attraction and alignement (not ready)
  //float dx = x_finish - x_start;
  //float dy = y_finish - y_start;
  //float alpha = atan2f(dx, dy) - psi;
  //float pd = perpendicular_distance(pos->x, pos->y, x_start, y_start, dx, dy);
  //float beta = (sign(x_finish - pos->x)*M_PI_2) - psi - atanf((y_finish - pos->y)/(x_finish - pos->x)) ;
  //float trajectory = (sfp.tr_y_ali * sinf(alpha) * expf(sfp.tr_l_ali - pd)) + (sfp.tr_y_att * sinf(beta) * expf((pd - (3 * sfp.tr_l_att)) / (sfp.tr_l_att + pd)));

  // compute fluctuation and wall reaction
  float fw = expf(-powf(swarm_fish.r_w / sfp.l_w, 2.f));
  float ow = (-1.0f)*sinf(swarm_fish.theta_w) * (1.f + sfp.e_w1 * cosf(swarm_fish.theta_w) + sfp.e_w2 * cosf(2.f * swarm_fish.theta_w));
  swarm_fish.f_w = fw;
  swarm_fish.f_fluct = sfp.fluct * (1.f - sfp.alpha * fw) * normal_random_gen();
  swarm_fish.f_wall = sfp.y_w * ow * fw;
  swarm_fish.step_size = (1.1f - fw) / 2.f;

  // compute alignement and attraction with other drones
  struct EnuCoor_f *pos_focal = NULL;
  float psi_focal = 0.f;
  //uint8_t id_focal = 0;
  struct EnuCoor_f *pos_current = NULL;
  float psi_current = 0.f;
  uint8_t id_current = 0;
  float min_distance = 1000000.f;
  float max_influence = 0.f;
  swarm_fish.f_ali = 0.f;
  swarm_fish.f_att = 0.f;

  for (uint8_t ac = 0; ac < NB_ACS; ac++) {
    if (ti_acs[ac].ac_id == AC_ID || ti_acs[ac].ac_id == 0) { continue; }
    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[ac].ac_id)) / 1000., 0.);
    if (delta_t < 1.f) {
      // compute if others position is not older than 1s
      id_current = ti_acs[ac].ac_id;
      pos_current = acInfoGetPositionEnu_f(id_current);
      psi_current = acInfoGetCourse(id_current);
      if (sfp.strategy == 2) {
        pos_focal = NULL;
        swarm_fish.f_ali = swarm_fish.f_ali + neighbor_alignement(pos, pos_current, psi, psi_current);
        swarm_fish.f_att = swarm_fish.f_att + neighbor_attraction(pos, pos_current, psi);
      }
      if (sfp.strategy == 1) {
        float influence = neighbor_influence(pos, pos_current ,psi ,psi_current);
        if (influence > max_influence) {
          max_influence = influence;
          //id_focal = id_current;
          pos_focal = pos_current;
          psi_focal = psi_current;
        }
      }
      if (sfp.strategy == 0) {
        float distance = distance_drone_to_drone(pos, pos_current);
        if (distance < min_distance) {
          min_distance = distance;
          //id_focal = id_current;
          pos_focal = pos_current;
          psi_focal = psi_current;
        }
      }
    }
  }
  if (pos_focal != NULL) {
    swarm_fish.f_ali = neighbor_alignement(pos, pos_focal, psi, psi_focal);
    swarm_fish.f_att = neighbor_attraction(pos, pos_focal, psi);
  }

  // compute heading variation
  float diff_heading = swarm_fish.f_fluct + swarm_fish.f_wall + swarm_fish.f_ali + swarm_fish.f_att;
  return diff_heading;
}

#define PRESCALE 16
/** runs the uav according to fish movement model using velocity control
 * @return true
 */
bool swarm_fish_velocity_run(void)
{
  static int counter = PRESCALE;
  counter = counter - (1 + (int)(swarm_fish.f_w * (PRESCALE - 1)));
  if (counter > 0) { return true; }
  counter = PRESCALE;
  float diff_heading = calculate_new_heading();
  float new_heading = swarm_fish.heading + diff_heading;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  swarm_fish.heading = new_heading;
  autopilot_guided_update(GUIDED_FLAG_XY_BODY | GUIDED_FLAG_XY_VEL, swarm_fish.velocity, 0.0f, -sfp.alt, swarm_fish.heading);
  swarm_fish.velocity = sfp.max_velocity;
#if SWARM_FISH_SYNC_SEND
  send_swarm_message();
#endif
  return true;
}


