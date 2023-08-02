/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric/gvf_parametric.h
 *
 * Guiding vector field algorithm for 2D and 3D parametric trajectories.
 */

#ifndef GVF_PARAMETRIC_H
#define GVF_PARAMETRIC_H

#define GVF_PARAMETRIC_GRAVITY 9.806

/*! Default gain kroll for tuning the "coordinated turn" */
#ifndef GVF_PARAMETRIC_CONTROL_KROLL
#define GVF_PARAMETRIC_CONTROL_KROLL 1
#endif

/*! Default gain kclimb for tuning the climbing setting point */
#ifndef GVF_PARAMETRIC_CONTROL_KCLIMB
#define GVF_PARAMETRIC_CONTROL_KCLIMB 1
#endif

/*! Default scale for the error signals */
#ifndef GVF_PARAMETRIC_CONTROL_L
#define GVF_PARAMETRIC_CONTROL_L 1.
#endif

/*! Default scale for w  */
#ifndef GVF_PARAMETRIC_CONTROL_BETA
#define GVF_PARAMETRIC_CONTROL_BETA 1.
#endif

/*! Default gain kpsi for tuning the alignment of the vehicle with the vector field */
#ifndef GVF_PARAMETRIC_CONTROL_KPSI
#define GVF_PARAMETRIC_CONTROL_KPSI 1
#endif

/*! Default on/off coordination */
#ifndef GVF_PARAMETRIC_COORDINATION_COORDINATION
#define GVF_PARAMETRIC_COORDINATION_COORDINATION 0
#endif

/*! Default gain kc for the coordination algorithm */
#ifndef GVF_PARAMETRIC_COORDINATION_KC
#define GVF_PARAMETRIC_COORDINATION_KC 0.1
#endif

/*! Default timeout for the neighbors' information */
#ifndef GVF_PARAMETRIC_COORDINATION_TIMEOUT
#define GVF_PARAMETRIC_COORDINATION_TIMEOUT 1500
#endif

/*! Default broadcasting time */
#ifndef GVF_PARAMETRIC_COORDINATION_BROADTIME
#define GVF_PARAMETRIC_COORDINATION_BROADTIME 200
#endif

/*! Default number of neighbors per aircraft */
#ifndef GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS
#define GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS 4
#endif

#ifndef GVF_PARAMETRIC_STEP_ADAPTATION
#define GVF_PARAMETRIC_STEP_ADAPTATION 1
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "gvf_parametric_low_level_control.h"
#include "gvf_adapted_step.h"

#include "trajectories/gvf_parametric_3d_ellipse.h"
#include "trajectories/gvf_parametric_3d_lissajous.h"
#include "trajectories/gvf_parametric_2d_trefoil.h"
#include "trajectories/gvf_parametric_3d_sin.h"
#include "trajectories/gvf_parametric_3d_growing_lissajou.h"
#include "trajectories/gvf_parametric_drift_ellipse.h"

/** @typedef gvf_parametric_con
* @brief Control parameters for the GVF_PARAMETRIC
* @param w Virtual coordinate from the parametrization of the trajectory
* @param delta_T Time between iterations needed for integrating w
* @param s Defines the direction to be tracked. It takes the values -1 or 1.
* @param k_roll Gain for tuning the coordinated turn.
* @param k_climb Gain for tuning the climbing setting point.
*/
typedef struct {
  float w;
  float delta_T;
  int8_t s;
  float k_roll;
  float k_climb;
  float k_psi;
  float L;
  float beta;
  float w_dot;
  float kx;
  float ky;
  float kz;
} gvf_parametric_con;

extern gvf_parametric_con gvf_parametric_control;

/** @typedef gvf_parametric_coord
* @brief Coordination parameters for the GVF_PARAMETRIC
* @param w Virtual coordinate from the parametrization of the trajectory
* @param delta_T Time between iterations needed for integrating w
* @param s Defines the direction to be tracked. It takes the values -1 or 1.
* @param k_roll Gain for tuning the coordinated turn.
* @param k_climb Gain for tuning the climbing setting point.
*/
typedef struct {
  int8_t coordination;
  float kc;
  uint16_t timeout;
  uint16_t broadtime;
} gvf_parametric_coord;

extern gvf_parametric_coord gvf_parametric_coordination;

typedef struct gvf_parametric_coord_nei_info
{
  int8_t nei_id;
  float w;
  float w_dot;
  float desired_dw;
  uint32_t delta_t;
} gvf_parametric_coord_nei_info;


struct gvf_parametric_coord_tab {
  gvf_parametric_coord_nei_info tableNei[GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS];
  float error_deltaw[GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS];
  uint32_t last_comm[GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS];
};

extern struct gvf_parametric_coord_tab gvf_parametric_coordination_tables;


// Parameters for the trajectories
enum trajectories_parametric {
  TREFOIL_2D = 0,
  ELLIPSE_3D = 1,
  LISSAJOUS_3D = 2,
  TORUS_3D_SURFACE = 3,
  SINUS_3D = 4,
  GROWING_LISSAJOU = 5,
  DRIFT_ELLIPSE = 6,
  NONE_PARAMETRIC = 255,
};

typedef struct {
  enum trajectories_parametric type;
  float p_parametric[16];
  float phi_errors[3];
} gvf_parametric_tra;

extern gvf_parametric_tra gvf_parametric_trajectory;

// Init function
extern void gvf_parametric_init(void);

// Control functions
extern void gvf_parametric_set_direction(int8_t);

extern void gvf_parametric_set_w_gain(float);

/**
 * @brief Define the affine component of the transformation to apply to the current trajectory
 * 
 * @param x x-axis shift
 * @param y y-axis shift
 * @param z z-axis shift
 */
void gvf_parametric_set_offset(float x, float y, float z);


/**
 * @brief Like gvf_parametric_set_offset, but uses a waypoint instead
 * 
 * @param wp Waypoint's ID
 */
void gvf_parametric_set_offset_wp(uint8_t wp);

/**
 * @brief Like gvf_parametric_set_offset_wp, but specify a different altitude
 * 
 * @param wp Waypoint's ID
 * @param alt Altitude
 */
void gvf_parametric_set_offset_wpa(uint8_t wp, float alt);

/**
 * @brief Define the rotational component (through its Euler's angles) of the transformation to apply to the current trajectory
 * 
 * @param rz First rotation around z-axis (in degrees)
 * @param ry Rotation around y-axis (in degrees)
 * @param rzbis Second rotation around z-axis (in degrees)
 * 
 * The resulting rotation is:
 * R_z(rzbis) ∘ R_y(ry) ∘ R_z(rz)
 */
void gvf_paremetric_set_euler_rot(float rz, float ry, float rzbis);

/**
 * @brief Define the rotational component (through its Cardan's angles) of the transformation to apply to the current trajectory
 * 
 * @param rx Rotation around x-axis (in degrees)
 * @param ry Rotation around y-axis (in degrees)
 * @param rz Rotation around z-axis (in degrees)
 * 
 * The resulting rotation is:
 * R_z(rz) ∘ R_y(ry) ∘ R_x(rx)
 */
void gvf_paremetric_set_cardan_rot(float rx, float ry, float rz);

/**
 * @brief Define the rotational component (through its normalized quaternion) of the transformation to apply to the current trajectory
 * 
 * q = w + x*i + y*j + z*k
 * 
 * @param x Corresponding quaternion value
 * @param y Corresponding quaternion value
 * @param z Corresponding quaternion value
 * @param w Corresponding quaternion value
 */
void gvf_paremetric_set_quaternion_rot(float x, float y, float z, float w);

/**
 * @brief Set the rotation such that the x-axis is aligned with the vector from wp1 to wp2
 * 
 * @param wp1 Source waypoint ID
 * @param wp2 Destination waypoint ID
 */
void gvf_parametric_set_wps_rot(uint8_t wp1, uint8_t wp2);

/**
 * @brief Like gvf_parametric_set_wps_rot, but consider wp1 = (0,0,0) (and take only wp2 as argument)
 * 
 * @param wp Direction's waypoint ID
 */
void gvf_parametric_set_wp_rot(uint8_t wp);

/**
 * @brief Define the affine transform through translation and Cardan's angles
 * 
 * Quaternion q is defined as:
 *  q = w + x*i + y*j + z*k
 * 
 * @param x x-axis shift
 * @param y y-axis shift
 * @param z z-axis shift
 * @param rx Rotation around x-axis (in degrees)
 * @param ry Rotation around y-axis (in degrees)
 * @param rz Rotation around z-axis (in degrees)
 * 
 * @see gvf_paremetric_set_cardan_rot
 */
void gvf_parametric_set_affine_tr(float x, float y, float z, float rx, float ry, float rz);

/**
 * @brief Define the affine transform through translation and quaternion
 * 
 * @param x x-axis shift
 * @param y y-axis shift
 * @param z z-axis shift
 * @param qx Quaternion coordinate
 * @param qy Quaternion coordinate
 * @param qz Quaternion coordinate
 * @param qw Quaternion coordinate
 * 
 * @see gvf_paremetric_set_quaternion_rot
 */
void gvf_parametric_set_affine_q_tr(float x, float y, float z, float qx, float qy, float qz, float qw);


/**
 * @brief Like gvf_parametric_set_affine_tr, but set the offset through a waypoint
 * 
 * @param wp Origin waypoint ID 
 * @param rx Rotation around x-axis (in degrees)
 * @param ry Rotation around y-axis (in degrees)
 * @param rz Rotation around z-axis (in degrees)
 */
void gvf_parametric_set_affine_tr_wp(uint8_t wp, float rx, float ry, float rz);

/**
 * @brief Like gvf_parametric_set_affine_tr_wp, but set the altitude manually
 * 
 * @param wp Origin waypoint ID 
 * @param alt Altitude
 * @param rx Rotation around x-axis (in degrees)
 * @param ry Rotation around y-axis (in degrees)
 * @param rz Rotation around z-axis (in degrees)
 */
void gvf_parametric_set_affine_tr_wpa(uint8_t wp, float alt, float rx, float ry, float rz);

/**
 * @brief Define the affine transform through the line passing through two waypoints,
 * such that the origin is at wp1, and the x-axis go through wp2
 * 
 * @param wp1 ID of the 'origin' waypoint
 * @param wp2 ID of the 'direction' waypoint
 */
void gvf_parametric_set_affine_tr_wps(uint8_t wp1, uint8_t wp2);


extern void gvf_parametric_control_2D(float, float, float, float, float, float, float, float);
extern void gvf_parametric_control_3d(float, float, float, float, float, float, float, float, float,
                                      float, float, float);

// Coordination functions
extern void gvf_parametric_coordination_send_w_to_nei(void);
extern void gvf_parametric_coordination_parseRegTable(uint8_t *buf);
extern void gvf_parametric_coordination_parseWTable(uint8_t *buf);

// 2D Trefoil
extern bool gvf_parametric_2D_trefoil_XY(float, float, float, float, float, float, float);
extern bool gvf_parametric_2D_trefoil_wp(uint8_t, float, float, float, float, float);

// 3D Ellipse
extern bool gvf_parametric_3d_ellipse_XYZ(float, float, float, float, float, float);
extern bool gvf_parametric_3d_ellipse_wp(uint8_t, float, float, float, float);
extern bool gvf_parametric_3d_ellipse_wp_delta(uint8_t, float, float, float, float);

// 3D Lissajous
extern bool gvf_parametric_3d_lissajous_XYZ(float, float, float, float, float, float, float, float, float, float, float,
    float, float);
extern bool gvf_parametric_3d_lissajous_wp_center(uint8_t, float, float, float, float, float, float, float, float,
    float, float, float);

// 3D Sinusoid

/**
 * @brief Set parameters for a 3D sinusoid trajectory
 * 
 * @param ay Oscillations' amplitude along the y-axis
 * @param freq_y Frequency along the y-axis (before multiplication by 2*PI)
 * @param phase_y Added phase to the y oscillations (in degrees)
 * @param az Oscillations' amplitude along the z-axis
 * @param freq_z Frequency along the z-axis (before multiplication by 2*PI)
 * @param phase_z Added phase to the z oscillations (in degrees)
 * 
 * @return true 
 */
bool gvf_parametric_3d_sin(float ay, float freq_y, float phase_y, float az, float freq_z, float phase_z);

/**
 * @brief Set parameters for a 3D sinusoid trajectory with affine transform relative to a point and z-axis rotation
 * 
 * @param xo Translation coordinate
 * @param yo Translation coordinate
 * @param zo Translation coordinate
 * @param alpha Rotation around z-axis, in degrees
 * @param ay Oscillations' amplitude along the y-axis
 * @param freq_y Frequency along the y-axis (before multiplication by 2*PI)
 * @param phase_y Added phase to the y oscillations (in degrees)
 * @param az Oscillations' amplitude along the z-axis
 * @param freq_z Frequency along the z-axis (before multiplication by 2*PI)
 * @param phase_z Added phase to the z oscillations (in degrees)
 * 
 * @return true 
 */
bool gvf_parametric_3d_sin_XYZa(float xo, float yo, float zo, float alpha,
                                float ay, float freq_y, float phase_y, float az, float freq_z, float phase_z);


// 3D linear growth Lissajou curve

/**
 * @brief Set parameters for a 3D linear growth Lissajou trajectory
 * 
 * @param ax Speed along the x-axis
 * @param ay Oscillations' amplitude along the y-axis
 * @param az Oscillations' amplitude along the z-axis
 * @param f_y Frequency along the y-axis (in radiants, i.e. before multiplication by 2*PI)
 * @param phi_y Phase for the y-oscillations
 * @param f_z Frequency along the z-axis (in radiants, i.e. before multiplication by 2*PI)
 * @param phi_z Phase for the z-oscillations
 * 
 * @return true 
 */
bool gvf_parametric_3d_growing_lissajou(float ax, float ay, float az, float f_y, float phi_y, float f_z, float phi_z);

// Drift ellipse

/**
 * @brief Set parameters for a 3D drifting ellipse trajectory
 * 
 * @param v_x Speed along the x-axis
 * @param a_x Oscillations' amplitude along the x-axis
 * @param a_y Oscillations' amplitude along the y-axis
 * @param freq Frequency (in radiants, i.e. before multiplication by 2*PI)
 * @param phi Phase 
 * 
 * @return true 
 */
bool gvf_parametric_3d_drift_ellipse(float v_x, float a_x, float a_y, float freq, float phase);


#ifdef __cplusplus
}
#endif


#endif // GVF_PARAMETRIC_H
