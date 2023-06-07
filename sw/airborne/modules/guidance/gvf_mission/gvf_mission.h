/*
 * Copyright (C) 2023 Mael Feurgard <maeL.feurgard@laas.fr>
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

/** @file "modules/guidance/gvf_mission/gvf_mission.h"
 * @author Mael Feurgard <maeL.feurgard@laas.fr>
 * A wrapper module to allow the use of GVF parametric trajectories in Mission mode.
 */

#ifndef GVF_MISSION_H
#define GVF_MISSION_H

#include "modules/mission/mission_common.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "stdio.h"

/**
 * @brief Set up to 9 low-level control parameters of the GVF parametric controller
 * This function IS a mission callback function (ID: 'GVFSC')
 * 
 * @param nb : From 1 to 9 
 * @param params : Values for the different internal control parameters (ordered as in the definition of `gvf_parametric_con`)
 * @param flag : UNUSED
 * 
 * @return false  (set only once, do not repeatedly call this function)
 * 
 * @see gvf_parametric_con
 */
extern bool gvf_mission_set_control(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);

/**
 * @brief Modify the internal parameters of the GVF parametric controller (effects depend on the subsequent trajectory used)7
 * This function IS a mission callback function (ID: 'GVFSP')
 * 
 * @param nb : Number of parameters in `params`
 * @param params : Parameters to be set in `gvf_parametric_trajectory.p_parametric`
 * @param flag : UNUSED
 * 
 * @return false (set only once, do not repeatedly call this function)
 */
extern bool gvf_mission_set_params(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);

/**
 * @brief Wrap a call to `gvf_parametric_set_affine_tr`; set the isometric transform applied to the trajectory (translation + rotation)
 * This function IS a mission callback function (ID: 'GVFST')
 * 
 * @param nb : UNUSED (must be 6)
 * @param params : An array with 6 elements, namely the x,y,z coordinates of the translation, and the x,y,z Cardan angles of the rotation
 * @param flag : UNUSED
 * 
 * @return false (set only once, do not repeatedly call this function)
 */
extern bool gvf_mission_set_transform(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);

/**
 * @brief Wrap a call to `gvf_parametric_set_affine_q_tr`; set the isometric transform applied to the trajectory (translation + rotation)
 * This function IS a mission callback function (ID: 'GVFSQ')
 * 
 * @param nb : UNUSED (must be 7)
 * @param params : An array with 7 elements, namely the x,y,z coordinates of the translation, and the qx,qy,qz,qw normalized quaternion coordinates
 * @param flag : UNUSED
 * 
 * @return false (set only once, do not repeatedly call this function)
 */
extern bool gvf_mission_set_q_transform(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);

/**
 * @brief Set the trajectory used for control
 * This function IS a mission callback function (ID: 'GVFTR')
 * 
 * @param nb : UNUSED (must be 1)
 * @param params : An array with only the first element set, namely the `enum trajectories_parametric` value of the wanted trajectory
 * @param flag : UNUSED
 * 
 * @return false (set only once, do not repeatedly call this function)
 * 
 * @note Some care may be needed... There must be a safer way than doing an implicit float-to-enum conversion...
 */
extern bool gvf_mission_set_trajectory(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);


/**
 * @brief Run the GVF parametric controller
 * This function IS a mission callback function (ID: 'GVFGO')
 * @param nb : Either 0,1 or 2
 * @param params : Array of floats. The first cell may contain a trajectory type, the second a limit to the parametric value `w`, the third a value for `w`
 * @param flag : UNUSED
 * 
 * If `nb` is 0, simply run the GVF parametric controller with the current parameters, again and again.
 * If `nb` is 1, re-set the trajectory type before running the controller
 * If `nb` is 2, re-set the trajectory type AND add a limit (in absolute value) to the parametric control value `w`
 * If `nb` is 3, re-set the trajectory type,add a limit (in absolute value) to the parametric control value `w`, AND set the value of `w`
 * 
 * @return true : in most cases (run the controller repeatedly)
 * @return false : If `nb` was set to 2, happen when the parametric control value exceed the given limit 
 *                 (Also happen if the trajectory type is not known)
 */
extern bool gvf_mission_run_trajectory(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag);

/**
 * @brief Init function : register all aforedefined mission callback function
 * 
 */
extern void gvf_mission_register(void);

#endif  // GVF_MISSION_H
