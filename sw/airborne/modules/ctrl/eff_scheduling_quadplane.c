/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger.fr>
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

/** @file "modules/ctrl/eff_scheduling_quadplane.c"
 * Interpolation of control effectivenss matrix of a quadplane
 */

#include "modules/ctrl/eff_scheduling_quadplane.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"

#include "modules/datalink/downlink.h"

// Airspeed at which pusher motor is on/off
#ifndef EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED 2.0f
#endif

// Airspeed at which vertical motors are on/off
#ifndef EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED 8.0f
#endif

#ifdef STABILIZATION_INDI_G1
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = {
  STABILIZATION_INDI_G1_ROLL,
  STABILIZATION_INDI_G1_PITCH,
  STABILIZATION_INDI_G1_YAW,
  STABILIZATION_INDI_G1_THRUST,
  STABILIZATION_INDI_G1_THRUST_X
};
#endif

void eff_scheduling_quadplane_init(void)
{
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
    }
  }
}

void eff_scheduling_quadplane_periodic(void)
{
  // calculate squared airspeed
  float airspeed = stateGetAirspeed_f();

  if (airspeed > EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED) {

    // turn off vertical motors
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 4; j++) {
        g1g2[i][j] = 0;
      }
    }

    // elevon function of airspeed
    float offset_airspeed = airspeed - EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED; //offset for start eff at zero!
    Bound(offset_airspeed, 0.0f, 30.0f);
    float airspeed2 = offset_airspeed * offset_airspeed;

    float roll_eff = EFF_SCHEDULING_QUADPLANE_ROLL * airspeed2;
    g1g2[0][4] = -roll_eff / INDI_G_SCALING; // elevon_right
    g1g2[0][5] = -roll_eff / INDI_G_SCALING; // elevon_left

    float pitch_eff = EFF_SCHEDULING_QUADPLANE_PITCH * airspeed2;
    g1g2[1][4] =  pitch_eff / INDI_G_SCALING; // elevon_right
    g1g2[1][5] = -pitch_eff / INDI_G_SCALING; // elevon_left

    //g1g2[4][6] = EFF_SCHEDULING_QUADPLANE_THRUST_X; // pusher motor ????
  }
  else {
    // turn on vertical motors
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 4; j++) {
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
      }
    }

    //Come back to motor control
    g1g2[0][4] = 0; // elevon_left
    g1g2[0][5] = 0; // elevon_right

    g1g2[1][4] = 0; // elevon_left
    g1g2[1][5] = 0; // elevon_right

    if (airspeed < EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED) {
      //g1g2[4][6] = 0; // pusher motor ????
    } else {
      //g1g2[4][6] = EFF_SCHEDULING_QUADPLANE_THRUST_X;
    }
  }
}

extern void eff_scheduling_quadplane_report(void)
{
  float f[6] = {
    g1g2[1][4], g1g2[1][5],
    g1g2[2][4], g1g2[2][5],
    g1g2[1][0], g1g2[2][0]
  };
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 6, f);
}

