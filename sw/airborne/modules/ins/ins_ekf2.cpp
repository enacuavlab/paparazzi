/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/ins/ins_ekf2.cpp
 *
 * INS based in the EKF2 of PX4
 *
 */

#define CONFIG_EKF2_EXTERNAL_VISION

#include "modules/ins/ins_ekf2.h"
#include "modules/nav/waypoints.h"
#include "modules/core/abi.h"
#include "stabilization/stabilization_attitude.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "EKF/ekf.h"
#include "math/pprz_isa.h"
#include "math/pprz_geodetic_wgs84.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"

/** For SITL and NPS we need special includes */
#if defined SITL && USE_NPS
#include "nps_autopilot.h"
#include <stdio.h>
#endif

static Ekf toto;

static parameters *ekf_params;                    ///< The EKF parameters

struct ekf2_t ekf2;                               ///< Local EKF2 status structure

void ins_ekf2_init(void)
{

  ekf_params = toto.getParamHandle();

  ekf_params->filter_update_interval_us = 0;
/*
  ekf_params->delay_max_ms = 0;
  ekf_params->imu_ctrl = 0;
*/
}

void ins_ekf2_update(void){};
void ins_ekf2_change_param(int32_t unk){};
void ins_ekf2_remove_gps(int32_t mode){};
void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf){};
void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t *buf){};
