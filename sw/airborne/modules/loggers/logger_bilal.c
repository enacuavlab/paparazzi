/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** @file modules/loggers/logger_bilal.c
 */

#include "modules/loggers/logger_bilal.h"
#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "modules/actuators/actuators.h"
#include "modules/energy/electrical.h"

static bool logger_bilal_started = false;

/** Write the log header line according to the enabled parts */
void logger_bilal_start(void)
{
  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile, "time");
    sdLogWriteLog(pprzLogFile, ",ref_x,ref_y,ref_z");
    sdLogWriteLog(pprzLogFile, ",pos_x,pos_y,pos_z");
    sdLogWriteLog(pprzLogFile, ",speed_x,speed_y,speed_z");
    sdLogWriteLog(pprzLogFile, ",accel_x,accel_y,accel_z");
    sdLogWriteLog(pprzLogFile, ",phi,theta,psi");
    sdLogWriteLog(pprzLogFile, ",p,q,r");
    for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
      sdLogWriteLog(pprzLogFile, ",act_%d", i);
    }
    for (unsigned int i = 0; i < ACTUATORS_DSHOT_NB; i++) {
      if (actuators_dshot_values[i].activated) {
        sdLogWriteLog(pprzLogFile, ",rpm_%i,V_%i,A_%i", i, i, i);
      }
    }
    sdLogWriteLog(pprzLogFile, ",V_bat,A_bat,E_bat,P_avg");
    sdLogWriteLog(pprzLogFile, ",mode,block,stage");
    sdLogWriteLog(pprzLogFile,"\n");

    logger_bilal_started = true;
  }
}


/** Log the values to file */
void logger_bilal_periodic(void)
{
  if (pprzLogFile == -1) {
    return;
  }

  if (!logger_bilal_started) {
    logger_bilal_start();
  }

  sdLogWriteLog(pprzLogFile, "%.5f", get_sys_time_float());

  struct FloatVect3 ref_pos;
  ref_pos.x = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x);
  ref_pos.y = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y);
  ref_pos.z = POS_FLOAT_OF_BFP(guidance_v.z_ref);
  sdLogWriteLog(pprzLogFile, ",%.2f,%.2f,%.2f", ref_pos.x, ref_pos.y, ref_pos.z);

  struct NedCoor_f *pos = stateGetPositionNed_f();
  sdLogWriteLog(pprzLogFile, ",%.2f,%.2f,%.2f", pos->x, pos->y, pos->z);

  struct NedCoor_f *speed = stateGetSpeedNed_f();
  sdLogWriteLog(pprzLogFile, ",%.2f,%.2f,%.2f", speed->x, speed->y, speed->z);

  struct NedCoor_f *accel = stateGetAccelNed_f();
  sdLogWriteLog(pprzLogFile, ",%.2f,%.2f,%.2f", accel->x, accel->y, accel->z);

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  sdLogWriteLog(pprzLogFile, ",%.3f,%.3f,%.3f", att->phi, att->theta, att->psi);

  struct FloatRates *rates = stateGetBodyRates_f();
  sdLogWriteLog(pprzLogFile, ",%.3f,%.3f,%.3f", rates->p, rates->q, rates->r);

  for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
    sdLogWriteLog(pprzLogFile, ",%d", actuators[i].pprz_val);
  }

  for (unsigned int i = 0; i < ACTUATORS_DSHOT_NB; i++) {
    if (actuators_dshot_values[i].activated) {
      sdLogWriteLog(pprzLogFile, ",%.2f,%.2f,%.2f", actuators_dshot_values[i].rpm,
          actuators_dshot_values[i].voltage, actuators_dshot_values[i].current);
    }
  }

  sdLogWriteLog(pprzLogFile, ",%.2f,%3.f,%.3f,%.3f", electrical.vsupply, electrical.current,
      electrical.energy, (float)electrical.avg_power / electrical.avg_cnt);

  sdLogWriteLog(pprzLogFile, ",%d,%d,%d", autopilot_get_mode(), nav_block, nav_stage);

  // end line
  sdLogWriteLog(pprzLogFile,"\n");
}

