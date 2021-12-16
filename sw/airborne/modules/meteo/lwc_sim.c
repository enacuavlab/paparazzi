/*
 * Copyright (C) 2018-2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
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
 * @file "modules/meteo/lwc_sim.c"
 * @author VERDU Titouan
 *
 * This module fetch the Liquid Water Content (LWC) given by the python agent basedon the MesoNh cloud simulation.
 * Since data are received from datalink, it can be used in real flight with a virtual cloud.
 */

#include "modules/meteo/lwc_sim.h"

#include "std.h"
#include "paparazzi.h"
#include "state.h"
#include "modules/nav/common_nav.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/mission/mission_common.h"
#include "math/pprz_geodetic_float.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"

// LWC threshold for cloud border
#ifndef LWC_BORDER_THRESHOLD
#define LWC_BORDER_THRESHOLD 0.05
#endif

// Type of data
#define LWC_SIM_RAW 0 // LWC value
#define LWC_SIM_BORDER 1 // crossing border

struct LWCSim lwc_sim;

static int IvytoInt(uint8_t *buffer)
{
  int i, res;
  res = 0;

  for (i = 0; i < DL_PAYLOAD_COMMAND_command_length(buffer); i++) {
    res = res * 10 + DL_PAYLOAD_COMMAND_command(dl_buffer)[i];
  }

  return res;
}

static float denormalized(int value)
{
  float res1 = (float)value / 255.0;
  float res = (float)res1 * 0.65;
  return res;
}

static inline void border_send_shot_position(void)
{
  static int16_t border_point_nr = 0;
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = stateGetHorizontalSpeedNorm_f() * 10;

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &border_point_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);

  border_point_nr++;
}


void lwc_sim_init(void)
{
  lwc_sim.value = 0.f;
  lwc_sim.pos.x = 0.f;
  lwc_sim.pos.y = 0.f;
  lwc_sim.pos.z = 0.f;
  lwc_sim.time = 0.f;
  lwc_sim.inside_cloud = false;
}


void lwc_sim_msg_callback(void)
{

  if (DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID) {
    uint32_t stamp = get_sys_time_usec();

    // get raw value from message
    int raw = IvytoInt(dl_buffer);
    lwc_sim.value = denormalized(raw);
    lwc_sim.pos = *stateGetPositionEnu_f();
    lwc_sim.time = get_sys_time_float();

    // test border crossing
    if (lwc_sim.value > LWC_BORDER_THRESHOLD && lwc_sim.inside_cloud == false) {
      border_send_shot_position();
      lwc_sim.inside_cloud = true;
    } else if (lwc_sim.value <= LWC_BORDER_THRESHOLD && lwc_sim.inside_cloud == true) {
      border_send_shot_position();
      lwc_sim.inside_cloud = false;
    }

    AbiSendMsgPAYLOAD_DATA(LWC_SIM_ID, stamp, LWC_SIM_RAW, sizeof(float), (uint8_t *)(&lwc_sim.value));
    uint8_t inside = (uint8_t) lwc_sim.inside_cloud;
    AbiSendMsgPAYLOAD_DATA(LWC_SIM_ID, stamp, LWC_SIM_BORDER, 1, &inside);
  }
}
