/*
 * Copyright (C) 2021 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric/gvf_parametric_surf.h
 *
 * Guiding vector field algorithm for 2D and 3D parametric trajectories
 * with two parameters.
 */

#ifndef GVF_PARAMETRIC_SURF_SURF_H
#define GVF_PARAMETRIC_SURF_SURF_H

#define GVF_PARAMETRIC_SURF_GRAVITY 9.806

/*! Default gain kroll for tuning the "coordinated turn" */
#ifndef GVF_PARAMETRIC_SURF_CONTROL_KROLL
#define GVF_PARAMETRIC_SURF_CONTROL_KROLL 1
#endif

/*! Default gain kclimb for tuning the climbing setting point */
#ifndef GVF_PARAMETRIC_SURF_CONTROL_KCLIMB
#define GVF_PARAMETRIC_SURF_CONTROL_KCLIMB 1
#endif

/*! Default scale for the error signals */
#ifndef GVF_PARAMETRIC_SURF_CONTROL_L
#define GVF_PARAMETRIC_SURF_CONTROL_L 0.1
#endif

/*! Default scale for w  */
#ifndef GVF_PARAMETRIC_SURF_CONTROL_BETA
#define GVF_PARAMETRIC_SURF_CONTROL_BETA 0.01
#endif

/*! Default gain kpsi for tuning the alignment of the vehicle with the vector field */
#ifndef GVF_PARAMETRIC_SURF_CONTROL_KPSI
#define GVF_PARAMETRIC_SURF_CONTROL_KPSI 1
#endif

/*! Default on/off coordination */
#ifndef GVF_PARAMETRIC_SURF_COORDINATION
#define GVF_PARAMETRIC_SURF_COORDINATION 0
#endif

/*! Default gains kc1 and kc2 for the coordination algorithm */
#ifndef GVF_PARAMETRIC_SURF_COORDINATION_KC1
#define GVF_PARAMETRIC_SURF_COORDINATION_KC1 0.01
#endif

#ifndef GVF_PARAMETRIC_SURF_COORDINATION_KC2
#define GVF_PARAMETRIC_SURF_COORDINATION_KC2 0.01
#endif

/*! Default timeout for the neighbors' information */
#ifndef GVF_PARAMETRIC_SURF_COORDINATION_TIMEOUT
#define GVF_PARAMETRIC_SURF_COORDINATION_TIMEOUT 1500
#endif

/*! Default broadcasting time */
#ifndef GVF_PARAMETRIC_SURF_COORDINATION_BROADTIME
#define GVF_PARAMETRIC_SURF_COORDINATION_BROADTIME 200
#endif

/*! Default number of neighbors per aircraft */
#ifndef GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS
#define GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS 4
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_parametric_surf/trajectories/gvf_parametric_surf_3d_torus.h"

/** @typedef gvf_parametric_surf_con
* @brief Control parameters for the GVF_PARAMETRIC_SURF
* @param w1 Virtual coordinate from the parametrization of the trajectory
* @param w2 Virtual coordinate from the parametrization of the trajectory
* @param delta_T Time between iterations needed for integrating w
* @param s1 Defines the direction to be tracked for w1. It takes the values -1 or 1.
* @param s2 Defines the direction to be tracked for w2. It takes the values -1 or 1.
* @param k_roll Gain for tuning the coordinated turn.
* @param k_climb Gain for tuning the climbing setting point.
*/
typedef struct {
  float w1;
  float w2;
  float delta_T;
  int8_t s1;
  int8_t s2;
  float k_roll;
  float k_climb;
  float k_psi;
  float L;
  float beta1;
  float beta2;
  float w1_dot;
  float w2_dot;
} gvf_parametric_surf_con;

extern gvf_parametric_surf_con gvf_parametric_surf_control;

/** @typedef gvf_parametric_surf_coord
* @brief Coordination parameters for the GVF_PARAMETRIC_SURF
* @param coordination If we want to coordinate on a surface
* @param kc1 Gain for the consensus w1
* @param kc2 Gain for the consensus w2
* @param timeout When we stop considering a neighbor if we have not heard from it
* @param broadtime Period for broadcasting w1 and w2
*/
typedef struct {
  int8_t coordination;
  float kc1;
  float kc2;
  uint16_t timeout;
  uint16_t broadtime;
} gvf_parametric_surf_coord;

extern gvf_parametric_surf_coord gvf_parametric_surf_coordination;

struct gvf_parametric_surf_coord_tab {
  float tableNei[GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS][8];
  float error_deltaw1[GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS];
  float error_deltaw2[GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS];
  uint32_t last_comm[GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS];
};

extern struct gvf_parametric_surf_coord_tab gvf_parametric_surf_coordination_tables;

// Parameters for the trajectories
enum trajectories_parametric_surf {
  TORUS_3D = 0,
  NONE_SURF_PARAMETRIC = 255,
};

typedef struct {
  enum trajectories_parametric type;
  float p_parametric[16];
  float phi_errors[3];
} gvf_parametric_surf_tra;

extern gvf_parametric_surf_tra gvf_parametric_surf_trajectory;

// Init function
extern void gvf_parametric_surf_init(void);

// Control functions
extern void gvf_parametric_surf_set_direction_s1(int8_t);
extern void gvf_parametric_surf_set_direction_s2(int8_t);
extern void gvf_parametric_surf_control_3D(float, float, float, float, float, float, float, float, float,
                                      float, float, float);

// Coordination functions
extern void gvf_parametric_surf_coordination_send_w_to_nei(void);
extern void gvf_parametric_surf_coordination_parseRegTable(uint8_t *buf);
extern void gvf_parametric_surf_coordination_parseWTable(uint8_t *buf);

// 3D Torus
extern bool gvf_parametric_surf_3D_torus_XY(float, float, float, float, float, float, float);
extern bool gvf_parametric_surf_3D_torus_wp(uint8_t, float, float, float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_PARAMETRIC_SURF_H
