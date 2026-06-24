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
 * @file modules/nav/extended_dubins/nav_extended_dubins.h
 *
 * Navigation module to follow Extended Dubins Paths
 * (Dubins Paths with added straights at start and end)
 */

#ifndef NAV_EXTENDED_DUBINS_H
#define NAV_EXTENDED_DUBINS_H

#include "std.h"
#include "dubins_common.h"

extern bool dubins_draw;
extern int dubins_draw_samples;
extern float extra_straight_length;
extern float end_of_straight_time;


void extended_dubins_set_start(float x, float y, float theta_deg);
void extended_dubins_set_start_wp(uint8_t wp, float theta_deg);
void extended_dubins_set_end(float x, float y, float a, float theta_deg);
void extended_dubins_set_end_wp(uint8_t wp, float theta_deg);
void extended_dubins_set_radius(float radius);
void extended_dubins_set_pathtype(DubinsType type, float extra);

void dubins_setup(void);
bool nav_extended_dubins_init(void);
bool nav_extended_dubins_track(void);

extern bool dubins_use_gvf;

#endif // NAV_EXTENDED_DUBINS_H