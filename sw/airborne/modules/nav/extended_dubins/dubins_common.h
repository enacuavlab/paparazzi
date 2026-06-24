/*
 * Copyright (C) 2026  ENAC, Mael Feurgard
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/extended_dubins/dubins_common.h
 *
 * Useful primitives related to Dubins paths
 */


#ifndef DUBINS_COMMON_H
#define DUBINS_COMMON_H

#include "std.h"

#define FIRST_INVALID_DUBINS_TYPE (4*8)

// Different possible types of Dubins paths as sequence of manoeuvres, where S means Straight, L for Left turn and R for Right turn
typedef enum
{
  RSR = 0,
  LSL = 1,
  RSL = 2,
  LSR = 3,
  RLR = 4,
  LRL = 5,
  SLS = 6,
  SRS = 7,

  S_RSR = 8+0,
  S_LSL = 8+1,
  S_RSL = 8+2,
  S_LSR = 8+3,
  S_RLR = 8+4,
  S_LRL = 8+5,
  S_SLS = 8+6,
  S_SRS = 8+7,

  RSR_S = 2*8+0,
  LSL_S = 2*8+1,
  RSL_S = 2*8+2,
  LSR_S = 2*8+3,
  RLR_S = 2*8+4,
  LRL_S = 2*8+5,
  SLS_S = 2*8+6,
  SRS_S = 2*8+7,

  S_RSR_S = 3*8+0,
  S_LSL_S = 3*8+1,
  S_RSL_S = 3*8+2,
  S_LSR_S = 3*8+3,
  S_RLR_S = 3*8+4,
  S_LRL_S = 3*8+5,
  S_SLS_S = 3*8+6,
  S_SRS_S = 3*8+7,

  DUBINS_NONE = FIRST_INVALID_DUBINS_TYPE
} DubinsType;

bool HasStartExtension(DubinsType t);
bool HasEndExtension(DubinsType t);
bool NotExtendedDubins(DubinsType t);
bool StartExtendedDubins(DubinsType t);
bool EndExtendedDubins(DubinsType t);
bool BothExtendedDubins(DubinsType t);
DubinsType BaseDubinsType(DubinsType t);
bool ValidExtendedDubins(DubinsType t);
const char* dubinsTypeStr(DubinsType t);

typedef struct
{
  float x,y,theta;
} Pose2D_t;

typedef struct
{
  Pose2D_t init_point;  // Initial point of the element
  float radius;         // Radius; if 0, it is a straight; if positive, it is a left turn, if negative a right turn
  float length;         // Element length
}  DubinsElement_t;

typedef struct
{
  DubinsType type;
  float radius;
  DubinsElement_t elements[5];
} ExtendedDubins_t;

typedef struct
{
  Pose2D_t start_p, end_p;      // Endpoint poses
  float length;                 // Length of the path to follow. Used as an hint to fix some Dubins turns.  
  float end_time;               // End times of the trajectory, with respect to GPS ToW. If non-positive, speed is not tuned.
  float target_alt;             // Target final altitude
  DubinsType type;              // Type of Dubins path to generate
  float radius;                 // Radius of the target Dubins path
  float extra;                  // Extra parameter if needed (extended Dubins path)
} DubinsPb_t;


/**
 * @brief Estimate the pose reached by following the given element for the given length
 * 
 * @param el      Dubins element (Straight or Circular turn, with initial pose)
 * @param length  Length for which to follow the element
 * @return Pose2D_t Pose reached
 */
Pose2D_t dubins_element_follow(DubinsElement_t *el, float length);

/**
 * @brief Estimate the velocity vector direction assuming unit speed
 * 
 * Attribute `theta` is unused in the output.
 * 
 * @param el      Dubins element (Straight or Circular turn, with initial pose)
 * @param length  Length for which to follow the element
 * @return Pose2D_t Unit velocity vector reached
 */
Pose2D_t dubins_element_velocity(DubinsElement_t *el, float length);

/**
 * @brief Estimate the acceleration vector assuming unit speed
 * 
 * Attribute `theta` is unused in the output.
 * 
 * @param el      Dubins element (Straight or Circular turn, with initial pose)
 * @param length  Length for which to follow the element
 * @return Pose2D_t Acceleration reached assuming unit speed
 */
Pose2D_t dubins_element_accel(DubinsElement_t *el, float length);

/**
 * @brief Pose reached by following the given element to its end
 * 
 * @param el    Dubins element (Straight or Circular turn, with initial pose)
 * @return Pose2D_t 
 */
Pose2D_t dubins_element_end(DubinsElement_t *el);

/**
 * @brief Given a Path planning problem with a Dubins type hint and expected length, rebuild the full path
 * 
 * @param pb Specified Dubins path planning problem
 * @return ExtendedDubins_t 
 */
ExtendedDubins_t fit_dubins(DubinsPb_t* pb);

/**
 * @brief Give the length traveled from the initial pose associated to the Dubins problem given the current time and expected speed
 * 
 * @param pb              Specified Dubins path planning problem
 * @param nominal_speed   Expected constant speed along the path (m/s)
 * @param now             Current time (usually as a GPS ToW, in s)
 * @return float          Expected traveled length
 */
float dubins_estimate_current_parameter(DubinsPb_t* pb, float nominal_speed, float now);

/**
 * @brief Given an sequence of Dubins elements and a length, compute which element is reached and returns the length to travel on this element
 * 
 * If the given length is greater than the total path length, whatever remaining length is returned
 * and the element index is set to the number of elements (past-the-post index). 
 * 
 * @param el      Array of Dubins elements
 * @param el_num  Input : Number of elements in the array
 *                Output: Element on which we are at the given length (is left unchanged if the given length is greater than total path length)
 * @param length  Length to travel along the path
 * @return float 
 */
float dubins_find_current_element(DubinsElement_t* el, int* el_num, float length);



#endif //DUBINS_COMMON_H