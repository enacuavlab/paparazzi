/*
 * Copyright (C) 2026 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
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

/**
 * @file modules/sensors/baro_logger.c
 *
 * This reads the values for pressure, temperature and GPS and logs them to the SD card.
 */


#include "baro_logger.h"

#include "std.h"
#include "modules/core/abi.h"
#include "modules/gps/gps.h"
#include "modules/loggers/sdlog_chibios.h"



/** ABI binding for absolute pressure
 */
#ifndef BARO_LOGGER_BARO_ABS_ID
#define BARO_LOGGER_BARO_ABS_ID ABI_BROADCAST
#endif
static abi_event pressure_abs_ev;

/** ABI binding for temperature
 */
#ifndef BARO_LOGGER_TEMPERATURE_ID
#define BARO_LOGGER_TEMPERATURE_ID ABI_BROADCAST
#endif
static abi_event temperature_ev;


static float last_pressure;
static bool pressure_pending;

static float pressure_sum;
static float temperature_sum;
static uint16_t samples_count;

static bool log_started;



static void pressure_abs_cb(
  uint8_t sender_id __attribute__((unused)),
  uint32_t stamp __attribute__((unused)),
  float pressure)
{
  // BARO_ABS published before TEMPERATURE.
  last_pressure = pressure;
  pressure_pending = true;
}

static void temperature_cb(
  uint8_t sender_id __attribute__((unused)),
  float temperature)
{
  if (!pressure_pending) {
    return;
  }

  pressure_sum += last_pressure;
  temperature_sum += temperature;
  samples_count++;

  pressure_pending = false;
}

void tawaki_baro_logger_init(void)
{
  last_pressure = 0.0f;
  pressure_pending = false;

  pressure_sum = 0.0f;
  temperature_sum = 0.0f;
  samples_count = 0;

  log_started = false;

  AbiBindMsgBARO_ABS(BARO_LOGGER_BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
  AbiBindMsgTEMPERATURE(BARO_LOGGER_TEMPERATURE_ID, &temperature_ev, temperature_cb);
}

void tawaki_baro_logger_periodic(void)
{
  if (samples_count == 0) {
    return;
  }

  const float pressure_avg = pressure_sum / (float)samples_count;
  const float temperature_avg = temperature_sum / (float)samples_count;

  pressure_sum = 0.0f;
  temperature_sum = 0.0f;
  samples_count = 0;
    
  if (pprzLogFile == -1) {
    return;
  }

  if (!log_started) {
    sdLogWriteLog(
      pprzLogFile,
      "TAWAKI_BARO: Time(us) Pres(Pa) Temp(degC) GPS_fix TOW(ms) Week "
      "Lat(1e7deg) Lon(1e7deg) HMSL(mm) gspeed(cm/s) "
      "course(1e7rad) climb(cm/s)\n"
    );
    log_started = true;
  }

  const uint32_t log_time_usec = get_sys_time_usec();

  //If there is a GPS fix, we log the GPS data, otherwise we log 0 for all GPS fields
  if(gps.fix >= GPS_FIX_3D ) {
    sdLogWriteLog(
      pprzLogFile,
      "tawaki_baro: %lu %9.4f %9.4f %u %lu %u %ld %ld %ld %u %ld %ld\n",
      (unsigned long)log_time_usec,
      pressure_avg,
      temperature_avg,
      (unsigned int)gps.fix,
      (unsigned long)gps.tow,
      (unsigned int)gps.week,
      (long)gps.lla_pos.lat,
      (long)gps.lla_pos.lon,
      (long)gps.hmsl,
      (unsigned int)gps.gspeed,
      (long)gps.course,
      (long)-gps.ned_vel.z
    );

  } else {
    sdLogWriteLog(
      pprzLogFile,
      "tawaki_baro: %lu %9.4f %9.4f %u %lu %u %ld %ld %ld %u %ld %ld\n",
      (unsigned long)log_time_usec,
      pressure_avg,
      temperature_avg,
      (unsigned int)gps.fix,
      (unsigned long)gps.tow,
      (unsigned int)gps.week,
      0L, // lat
      0L, // lon
      0L, // hmsl
      0U, // gspeed
      0L, // course
      0L  // climb
    );  
  }
}

