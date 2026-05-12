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
 * @file modules/nav/nav_extended_dubins.h
 *
 * Navigation module to follow Extended Dubins Paths
 * (Dubins Paths with added straights at start and end)
 */

#ifndef NAV_EXTENDED_DUBINS_H
#define NAV_EXTENDED_DUBINS_H

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

  NONE = FIRST_INVALID_DUBINS_TYPE
} DubinsType;

bool HasStartExtension(DubinsType t);
bool HasEndExtension(DubinsType t);
bool NotExtendedDubins(DubinsType t);
bool StartExtendedDubins(DubinsType t);
bool EndExtendedDubins(DubinsType t);
bool BothExtendedDubins(DubinsType t);
DubinsType BaseDubinsType(DubinsType t);
bool ValidExtendedDubins(DubinsType t);

extern bool dubins_draw;

typedef struct
{
  float x,y,theta;
} Pose2D_t;

// Data recovered from CUSTOM_MISSION of type DUBIN
typedef struct
{
  Pose2D_t start_p, end_p;      // Endpoint poses
  float start_time;   // Start and end times of the trajectory, with respect to GPS ToW. If end time is non-positive, speed is not tuned.
  float end_time;
  float target_alt;             // Target final altitude
  DubinsType type;              // Type of Dubins path to generate
  float radius;                 // Radius of the target Dubins path
  float extra;                  // Extra parameter if needed (extended Dubins path)
} DubinsPb_t;

typedef struct
{
  Pose2D_t init_point;  // Initial point of the element
  float radius;         // Radius; if 0, it is a straight; if positive, it is a left turn, if negative a right turn
  float length;         // Element length
}  DubinsElement_t;

void extended_dubins_set_start(float x, float y, float theta_deg);
void extended_dubins_set_start_wp(uint8_t wp, float theta_deg);
void extended_dubins_set_end(float x, float y, float a, float theta_deg);
void extended_dubins_set_end_wp(uint8_t wp, float theta_deg);
void extended_dubins_set_radius(float radius);
void extended_dubins_set_pathtype(DubinsType type, float extra);

void dubins_setup(void);
bool nav_extended_dubins_init(void);
bool nav_extended_dubins_track(void);

#endif // NAV_EXTENDED_DUBINS_H