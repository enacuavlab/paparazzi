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
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "generated/flight_plan.h"
#include "modules/datalink/telemetry.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * Default parameters
 */

#ifndef SWARM_FISH_NB_NEIGHBORS
#define SWARM_FISH_NB_NEIGHBORS 2
#endif

#ifndef SWARM_FISH_INTRUDER_ID
#define SWARM_FISH_INTRUDER_ID 0
#endif

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
  float heading;                ///< heading command
  float theta_obs;              ///< angle to wall
  struct FloatVect3 f_social;   ///< social interactions
  struct FloatVect3 f_nav;      ///< navigation
  struct FloatVect3 f_intruder; ///< intruder repulsion
  struct FloatVect3 f_obstacle; ///< obstacle repulsion
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

  swarm_fish_params.nb_neighbors = SWARM_FISH_NB_NEIGHBORS;
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
 * @return anisotropic gain in [-1., 1.]
 */
static float anisotropic(float a, float psi)
{
  return (1.f + a * cosf(psi)) / (1.f + a);
}

/** Calculates distance between the uav and the arena wall
 *  centered on a WP
 *
 * @param[in] wp_id center of the arena waypoint ID
 * @param[in] radius radius of the arena [m]
 * @param[in] pos agent position
 * @return distance to circular wall, negatif if outside
 */
static float distance_to_arena(uint8_t wp_id, float radius, struct EnuCoor_f *pos)
{
  float x_center = waypoint_get_x(wp_id);
  float y_center = waypoint_get_y(wp_id);
  float dist = radius - sqrtf(((pos->x - x_center) * (pos->x - x_center)) + ((pos->y - y_center) * (pos->y - y_center)));
  return dist;
}

/** calculates the relative orientation to the wall
 *
 * @param[in] pos agent position
 * @param[in] course agent course
 * @return angle to circular wall
 */
static float angle_to_wall(struct EnuCoor_f *pos, float course)
{
  float theta = atan2f(pos->y, pos->x);
  float delta = course - theta;
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

/** returns course angle in ENU frame
 *  it can be the ground course (GPS) or the heading (mag) depending on
 *  the use_heading flag
 *
 * @param use_heading true to use heading instead of ground course
 * @return course angle
 */
static float get_course(bool use_heading)
{
  if (use_heading) {
    return M_PI_2 - stateGetNedToBodyEulers_f()->psi;
  } else {
    return M_PI_2 - stateGetHorizontalSpeedDir_f();
  }
}

/** calculates a uav's viewing angle on another uav
 * @param pos ENU coordinates of the viewer drone
 * @param other ENU coordinates of the viewed drone
 * @param use_heading heading flag (use ground_course or heading)
 * @return viewing angle
 */
static float get_viewing_angle(struct EnuCoor_f *pos, struct EnuCoor_f *other, bool use_heading)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  float dir = atan2f(diff.y, diff.x);
  float angle = dir - get_course(use_heading);
  FLOAT_ANGLE_NORMALIZE(angle);
  return angle;
}

/** square function */
static inline float squared(float x)
{
  return x * x;
}

/** calculates difference between two course angle
 * @param course first course
 * @param course_other second course
 * @return the difference between the two angles
 */
static float get_course_diff(float course, float course_other)
{
  float da = course_other - course;
  FLOAT_ANGLE_NORMALIZE(da);
  return da;
}

/** Social interaction
 *
 * @param[in] pos agent position
 * @param[in] vel agent velocity
 * @param[in] course agent course
 * @param[in] other neighbor position
 * @param[in] vel_other neighbor velocity
 * @param[in] course_other neighbor course
 * @param[in] p pointer to swarm parameters
 * @param[out] influence total influence of the considered neighbor
 * @return the acceleration related to social interaction with a neighbor
 */
static struct FloatVect3 interaction_social(
    struct EnuCoor_f *pos, struct EnuCoor_f *vel, float course,
    struct EnuCoor_f *other, struct EnuCoor_f *vel_other, float course_other,
    struct SwarmFishParams *p,
    float *influence)
{
  struct FloatVect3 acc = {0};
  const float dij = get_distance_3D(pos, other);
  if (dij < 1e-3) {
    return acc; // too close to do something, better avoid a division by zero later
  }
  const float dphi = get_course_diff(course, course_other);
  const float psi = get_viewing_angle(pos, other, p->use_heading);
  const float c_psi = cosf(psi);
  const float s_psi = sinf(psi);
  const float c_dphi = cosf(dphi);
  const float s_dphi = sinf(dphi);
  const float delta_z = other->z - pos->z;
  const float v_h_i = FLOAT_VECT2_NORM(*vel);
  const float v_h_j = FLOAT_VECT2_NORM(*vel_other);

  // attraction
  const float f_att = p->y_att * ((dij - p->d_att) / p->d_att) / (1.f + squared(dij / p->l_att)) * anisotropic(p->a_att, psi);
  // alignement
  const float f_ali = p->y_ali * expf(-squared(dii / p->l_ali)) * s_dphi * anisotropic(p->a_ali, psi);
  // vertical
  const float f_z = p->y_z * ((delta_z - sign(delta_z) * p->d_z) / p->d_z) * expf(-squared(dij / p->l_z)) * anisotropic(p->a_z, psi);
  // speed
  const float f_v_base = p->y_v * expf(-squared(dij / p->l_v)) * anisotropic(p->a_v, psi) / (float)(Max(p->nb_neighbors, 1));
  const float f_v_x = f_v_base * (v_h_j * c_phi - v_h_i);
  const float f_v_y = f_v_base * (v_h_j * s_sphi);
  const float f_v_z = f_v_base * (vel_other->z - vel->z);

  acc->x = f_att * c_psi + f_v_x;
  acc->y = f_att * s_psi + f_ali + f_v_y;
  acc->z = f_z + f_v_z;

  // FIXME is it really what we want here, or just the norm of the acceleration ?
  *influence = sqrtf(squared(f_att) + squared(f_ali) + squared(f_z) + squared(f_v_x) + squared(f_v_y) + squared(f_v_z));

  return acc;
}

/** Navigation interaction
 *
 * @param[in] pos agent position
 * @param[in] vel agent velocity
 * @param[in] course agent course
 * @param[in] p pointer to swarm parameters
 * @param[in] n pointer to swarm navigation setpoint
 * @return the acceleration related to the navigation setpoints
 */
static struct FloatVect3 interaction_nav(
    struct EnuCoor_f *pos, struct EnuCoor_f *vel, float course,
    struct SwarmFishParams *p, struct SwarmFishNav *n)
{
  float phi_nav = n->course;
  if (p->free_heading) {
    // free flight, nav direction is current course
    phi_nav = course;
  }
  float vz_nav = 0.f;
  if (!p->vz_ctl) {
    // if not vz control, compute vz_nav from altitude setpoint
    vz_nav = p->y_z_nav * tanhf((n->alt - pos->z) / p->l_z_nav);
  }
  const float h_speed = FLOAT_VECT2_NORM(*vel);
  struct FloatVect3 acc;
  acc->x = p->y_nav * (n->speed * cosf(phi_nav - get_course(p->use_heading)) - h_speed);
  acc->y = p->y_nav * (n->speed * sinf(phi_nav - get_course(p->use_heading)) - h_speed);
  acc->z = p->y_nav * (vz_nav - vel->z);

  return acc;
}

/** Intruder interaction
 *
 * @param[in] pos agent position
 * @param[in] vel agent velocity
 * @param[in] course agent course
 * @param[in] other neighbor position
 * @param[in] vel_other neighbor velocity
 * @param[in] course_other neighbor course
 * @param[in] p pointer to swarm parameters
 * @return the acceleration related to intruder avoidance
 */
static struct FloatVect3 interaction_intruder(
    struct EnuCoor_f *pos, struct EnuCoor_f *vel, float course,
    struct EnuCoor_f *other, struct EnuCoor_f *vel_other, float course_other,
    struct SwarmFishParams *p)
{
  struct FloatVect3 acc = {0};
  const float dij = get_distance_3D(pos, other);
  if (dij < 1e-3) {
    return acc; // too close to do something, better avoid a division by zero later
  }

  // horizontal repulsion
  const float psi = get_viewing_angle(pos, other, p->use_heading);
  const float psi_int = get_viewing_angle(other, pos, p->use_heading);
  const float speed = FLOAT_VECT2_NORM(*vel);
  const float speed_int = FLOAT_VECT2_NORM(*vel_other);
  const float relative_vel_ratio = (speed * cosf(psi) + speed_int * cosf(psi_int)) / (Max(speed + speed_int, 0.1f));

  const float f_rep = p->y_in * expf(-squared(d / p->l_in)) * (1.f + p->a_in * relative_vel_ratio) / (1.f + p->a_in);
  acc->x = - f_rep * cosf(psi);
  acc->y = - f_rep * sinf(psi);

  // vertical repulsion
  const delta_z = other->z - pos->z;
  acc-z = - p->y_z_in * tanhf(delta_z / p->d_z_in) * expf(-squared(dij / p->l_z_in));

  return acc;
}

/** Obstacle interaction
 *
 * @param[in] pos agent position
 * @param[in] vel agent velocity
 * @param[in] dist distance to obstacle
 * @param[in] angle viewing angle of the obstacle
 * @param[in] p pointer to swarm parameters
 * @return the acceleration related to horizontal obstacle avoidance
 */
static struct FloatVect3 interaction_obstacle(
    struct EnuCoor_f *pos, struct EnuCoor_f *vel,
    float dist, float angle,
    struct SwarmFishParams *p)
{
  const float speed = FLOAT_VECT2_NORM(*vel);
  const float collision_speed = speed * cosf(angle);

  const float f_obs = p->y_obs * expf(-(dist / p->l_obs)) * expf(p->y_v_obs * collision_speed / p->v0_obs) * anisotropic(p->a_obs, angle);
  struct FloatVect3 acc;
  acc->x = - f_obs * cosf(angle);
  acc->y = f_obs * sinf(angle);
  acc->z = 0.f;
  return acc;
}

/** Vertical boundaries interactions
 *
 * @param[in] pos agent position
 * @param[in] dist distance to obstacle
 * @param[in] angle viewing angle of the obstacle
 * @param[in] p pointer to swarm parameters
 * @return the acceleration related the floor and ceiling boundaries
 */
static struct FloatVect3 interaction_vertical_boundaries(
    struct EnuCoor_f *pos,
    float z_min, float z_max,
    struct SwarmFishParams *p)
{
  struct FloatVect3 acc = {0};
  if (!isnan(z_min)) {
    const float delta_z = pos->z - zmin;
    acc.z += p->y_z_obs * expf(-delta_z / p->l_z_obs);
  }
  if (!isnan(z_max)) {
    const float delta_z = z_max - pos->z;
    acc.z += -p->y_z_obs * expf(-delta_z / p->l_z_obs);
  }
  return acc;
}

/** Compute the total interactions with neighbors and environement
 */
static void compute_interactions(struct SwarmFish *swarm, struct SwarmFishParams *p, struct SwarmFishNav *n)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct EnuCoor_f *vel = stateGetSpeedEnu_f();
  float course = get_course(p->use_heading);

  // TODO more obstacles than the circular arena
  float wall_dist = distance_to_arena(SWARM_FISH_ARENA_WP, p->arena_radius, pos);
  float wall_angle = angle_to_wall(pos, course);

  // We need to store results since we keep only the most influentials
  struct FloatVect3 socials[NB_ACS];
  float social_influences[NB_ACS] = {0};
  for (uint8_t ac = 0; ac < NB_ACS; ac++) {
    // use if AC is registered
    if (ti_acs[ac].ac_id == AC_ID || ti_acs[ac].ac_id == 0 || ti_acs[ac].ac_id == p->intruder_id) { continue; }

    // use if AC position is not older than 1s
    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[ac].ac_id)) / 1000.f, 0.f);
    if (delta_t > 1.f) { continue; }

    uint8_t id_other = ti_acs[ac].ac_id;
    struct EnuCoor_f *pos_other = acInfoGetPositionEnu_f(id_other);
    struct EnuCoor_f *vel_other = acInfoGetVelocityEnu_f(id_other);
    float course_other = acInfoGetCourse(id_other);
    socials[ac] = interaction_social(pos, vel, course, pos_other, vel_other, course_other, p, &social_influences[ac]);
  }
  // Sum social interactions of the 'nb_neighbors' most influential.
  // Since we don't sort the results, just go through the values
  // as many times as needed.
  FLOAT_VECT3_ZERO(swarm->f_social);
  for (uint8_t i = 0; i < p->nb_neighbors; i++) {
    float best_inf = 0.f;
    uint8_t best_idx = 0;
    for (uint8_t ac = 0; ac < NB_ACS; ac++) {
      if (social_influences[ac] > best_inf) {
        best_inf = social_influences[ac];
        best_idx = ac;
      }
    }
    VECT3_ADD(swarm->f_social, socials[best_idx]);
    social_influences[best_idx] = -1.f; // will be skipped at next loop
  }

  // navigation
  swarm->f_nav = interaction_nav(pos, vel, course, p, n);

  // intruder
  uint8_t ti_int = ti_acs_id[p->intruder_id];
  if (p->intruder_id != 0 && ti_acs[ti_int].ac_id != 0) {
    // use if AC position is not older than 1s
    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[ti_int].ac_id)) / 1000.f, 0.f);
    if (delta_t > 1.f) { continue; }
    struct EnuCoor_f *pos_int = acInfoGetPositionEnu_f(p->intruder_id);
    struct EnuCoor_f *vel_int = acInfoGetVelocityEnu_f(p->intruder_id);
    float course_int = acInfoGetCourse(p->intruder_id);
    swarm->f_intruder = interaction_intruder(pos, vel, course, pos_int, vel_int, course_int, p);
  }

  // obstacles
}


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


