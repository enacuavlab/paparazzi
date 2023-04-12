/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file "modules/ctrl/eff_scheduling_colibri.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Interpolation of control effectivenss matrix of the Colibri.
 */

#include "modules/ctrl/eff_scheduling_colibri.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "state.h"

// Airspeed at which only with motor
#ifndef INDI_SCHEDULING_LOW_AIRSPEED
#define INDI_SCHEDULING_LOW_AIRSPEED 8.0
#endif

#ifndef INDI_SCHEDULING_HIGH_AIRSPEED
#define INDI_SCHEDULING_HIGH_AIRSPEED 15.0
#endif


#if INDI_NUM_ACT < 6
#error "This module works with Colibri with 8 actuators by only 6 are managed by INDI "
#endif

static float g_forward[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL_FWD, STABILIZATION_INDI_G1_PITCH_FWD, STABILIZATION_INDI_G1_YAW_FWD, STABILIZATION_INDI_G1_THRUST_FWD};

static float g_hover[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL, STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST};



void ctrl_eff_scheduling_init(void)
{
  for (uint8_t i = 0; i < 4; i++) {
    g_hover[0][i] = g_hover[0][i] / INDI_G_SCALING;
    g_hover[1][i] = g_hover[1][i] / INDI_G_SCALING;
    g_hover[2][i] = g_hover[2][i] / INDI_G_SCALING;
    g_hover[3][i] = g_hover[3][i] / INDI_G_SCALING;

    g_forward[0][i] = g_forward[0][i] / INDI_G_SCALING;
    g_forward[1][i] = g_forward[1][i] / INDI_G_SCALING;
    g_forward[2][i] = g_forward[2][i] / INDI_G_SCALING;
    g_forward[3][i] = g_forward[3][i] / INDI_G_SCALING;
  }
}

void ctrl_eff_scheduling_periodic(void)
{
  // freq = 20.0 Hz
  float airspeed = stateGetAirspeed_f();

  float ratio = 1/(INDI_SCHEDULING_HIGH_AIRSPEED - INDI_SCHEDULING_LOW_AIRSPEED)*airspeed - INDI_SCHEDULING_LOW_AIRSPEED/(INDI_SCHEDULING_HIGH_AIRSPEED - INDI_SCHEDULING_LOW_AIRSPEED);

  Bound(ratio,0.0,1.0);

  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < 4; j++) {
       // Motor
      g1g2[i][j] = g_hover[i][j] * (1.0 - ratio) + g_forward[i][j] * ratio;
    }
  }

  // calculate squared airspeed
  Bound(airspeed, 0.0, 30.0);
  float airspeed2 = airspeed*airspeed;

  float roll_eff = CE_ROLL_A*airspeed2;
  g1g2[0][4] = -roll_eff/1000;
  g1g2[0][5] =  roll_eff/1000;

  float pitch_eff = CE_PITCH_A*airspeed2;
  g1g2[1][4] = -pitch_eff/1000;
  g1g2[1][5] =  pitch_eff/1000;

  float yaw_eff = CE_YAW_A*airspeed2;
  g1g2[2][4] = -yaw_eff/1000;
  g1g2[2][5] = -yaw_eff/1000;

  g1g2[3][4] = 0;
  g1g2[3][5] = 0;
}


