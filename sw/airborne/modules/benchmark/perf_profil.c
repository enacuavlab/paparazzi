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

void perf_profil_log(char * msg)
{
  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile, "PPT %s %lu\n", msg, get_sys_time_usec());
  }
}


