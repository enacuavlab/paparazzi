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

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
#endif

static Ekf ekf;

static parameters *ekf_params;                    ///< The EKF parameters

struct ekf2_t ekf2;                               ///< Local EKF2 status structure

void ins_ekf2_init(void)
{
  ekf_params = ekf.getParamHandle();
}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)),
                            uint32_t stamp,
                            int32_t flow_x,
                            int32_t flow_y,
                            int32_t flow_der_x __attribute__((unused)),
                            int32_t flow_der_y __attribute__((unused)),
                            float quality,
                            float size_divergence __attribute__((unused)))
{
  flowSample sample;
  sample.time_us = stamp;

  // Wait for two measurements in order to integrate
  if (ekf2.flow_stamp <= 0) {
    ekf2.flow_stamp = stamp;
    return;
  }

  // Calculate the timestamp
//  sample.dt = (stamp - ekf2.flow_stamp);
  ekf2.flow_stamp = stamp;

  /* Build integrated flow and gyro messages for filter
  NOTE: pure rotations should result in same flow_x and
  gyro_roll and same flow_y and gyro_pitch */
  Vector2f flowdata;
  flowdata(0) = RadOfDeg(flow_y) ;//* (1e-6 *
                                    //sample.dt);                       // INTEGRATED FLOW AROUND Y AXIS (RIGHT -X, LEFT +X)
  flowdata(1) = - RadOfDeg(flow_x); // * (1e-6 *
                                    //sample.dt);                     // INTEGRATED FLOW AROUND X AXIS (FORWARD +Y, BACKWARD -Y)

  sample.quality = quality;                     // quality indicator between 0 and 255
//  sample.flow_xy_rad =
 //   flowdata;                   // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
 // sample.gyro_xyz = Vector3f{NAN, NAN, NAN};    // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive

  // Update the optical flow data based on the callback
  //ekf.setOpticalFlowData(sample);
  ekf.setOpticalFlowData(sample);
}
#endif

void ins_ekf2_update(void){};
void ins_ekf2_change_param(int32_t unk){};
void ins_ekf2_remove_gps(int32_t mode){};
void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf){};
void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t *buf){};
