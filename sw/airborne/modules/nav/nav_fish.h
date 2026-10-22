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

/** @file "modules/nav/swarm_fish.h"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Bio-inspired swarm navigation
 */

#ifndef SWARM_FISH_H
#define SWARM_FISH_H

#include "std.h"

/**
 * nav fish param structure
 */
struct SwarmFishParams {
  // Attraction parameters
  float y_att;        ///< attraction intensity
  float l_att;        ///< attraction length
  float d_att;        ///< horizontal equilibrium distance
  float a_att;        ///< attraction anisotropy (0 for isotropic behavior)
  // Alignment parameters
  float y_ali;        ///< alignement intensity
  float l_ali;        ///< alignement length
  float a_att;        ///< alignement anisotropy (0 for isotropic behavior)
  // Speed parameters
  float y_v;          ///< speed adaptation intensity
  float l_v;          ///< speed adaptation length
  float a_v;          ///< speed adaptation anisotropy (0 for isotropic behavior)
  // Vertical parameters
  float y_z;          ///< vertical intensity
  float l_z;          ///< vertical length
  float d_z;          ///< vertical equilibrium distance
  float a_z;          ///< vertical anisotropy (0 for isotropic behavior)
  // Navigation parameters
  float y_nav;        ///< navigation intensity (inner speed setpoint loop)
  float y_z_nav;      ///< navigation altitude intensity (outer loop)
  float l_z_nav;      ///< navigation altitude length (outer loop)
  // Intruder parameters
  float y_in;         ///< intruder horizontal repulsion intensity
  float l_in;         ///< intruder horizontal repulsion length
  float a_in;         ///< intruder horizontal repulsion anisotropy (0 for isotropic behavior)
  float y_z_in;       ///< intruder vertical repulsion intensity
  float l_z_in;       ///< intruder vertical repulsion length
  float d_z_in;       ///< intruder vertical repulsion distance
  // Obstacles parameters
  float y_obs;        ///< obstacle horizontal repulsion intensity
  float y_v_obs;      ///< obstacle horizontal repulsion speed adaptation
  float l_obs;        ///< obstacle horizontal repulsion length
  float a_obs;        ///< obstacle horizontal repulsion anisotropy (0 for isotropic behavior)
  float v0_obs;       ///< obstacle horizontal repulsion speed normalization
  float y_z_obs;      ///< obstacle vertical repulsion intensity
  float l_z_obs;      ///< obstacle vertical repulsion length
  // Boundaries, limits and nav settings
  float max_speed;    ///< max horizontal speed
  float min_speed;    ///< min horizontal speed
  float max_yaw;      ///< max yaw rate
  float max_alt;      ///< max altitude
  float min_alt;      ///< min altitude
  float arena_radius; ///< radius of flight arena (<0 to disable)
  bool use_heading;   ///< use heading or ground course mode
  bool free_heading;  ///< free flight mode
  bool vz_ctl;        ///< use vz instead of alt for vertical control
};

/**
 * nav fish navigation setpoints and limits
 */
struct SwarmFishNav {
  // Navigation setpoints
  float speed;        ///< target swarm speed [m/s]
  float alt;          ///< target altitude [m]
  float vz;           ///< target vertical speed [m/s]
  float course;       ///< target ground course [rad]
};

extern struct SwarmFishParams swarm_fish_params;
extern struct SwarmFishNav swarm_fish_nav;

/**
 * nav fish init
 */
extern void swarm_fish_init(void);

/**
 * nav fish velocity run
 */
extern bool swarm_fish_velocity_run(void);

#endif  // SWARM_FISH_H

