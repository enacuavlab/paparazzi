/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/benchmark/perf_profil.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Performance profiling tool
 */

#include "modules/benchmark/perf_profil.h"
#include "mcu_periph/sys_time.h"
#include "modules/loggers/sdlog_chibios.h"
#include <hal.h>
#include <ch.h>

void perf_profil_log(char * msg)
{
  if (pprzLogFile != -1) {
    uint32_t t = RTC2US(STM32_SYSCLK, chSysGetRealtimeCounterX());
    sdLogWriteLog(pprzLogFile, "PPT %s %lu\n", msg, t);
  }
}

#ifndef PERF_PROFIL_EVENT_MAX
#define PERF_PROFIL_EVENT_MAX 1000
#endif
static uint32_t nb_event = 0;
static uint32_t t_start = 0;
static uint32_t dt_start = 0;
static uint32_t dt_end = 0;
static uint32_t dt_min = 0xFFFFFFFF;
static uint32_t dt_max = 0;

void perf_profil_event_start(void)
{
  dt_start = chSysGetRealtimeCounterX();
  if (nb_event == 0) {
    t_start = dt_start;
  }
}

void perf_profil_event_end(void)
{
  dt_end = chSysGetRealtimeCounterX();
  nb_event += 1;
  uint32_t dt = dt_end - dt_start;
  if (dt < dt_min) {
    dt_min = dt;
  }
  if (dt > dt_max) {
    dt_max = dt;
  }
  if (nb_event >= PERF_PROFIL_EVENT_MAX) {
    sdLogWriteLog(pprzLogFile, "PPT event %lu %lu %lu %lu\n",
        nb_event,
        RTC2US(STM32_SYSCLK, dt_end - t_start) / nb_event,
        RTC2US(STM32_SYSCLK, dt_min),
        RTC2US(STM32_SYSCLK, dt_max));
    nb_event = 0;
    dt_min = 0xFFFFFFFF;
    dt_max = 0;
  }
}

