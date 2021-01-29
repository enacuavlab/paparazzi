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

/** @file "modules/benchmark/perf_profil.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Performance profiling tool
 */

#ifndef PERF_PROFIL_H
#define PERF_PROFIL_H

extern void perf_profil_log(char * msg);
extern void perf_profil_event_start(void);
extern void perf_profil_event_end(void);

#ifdef PPRZ_PERF_TRACE
#error "PPRZ_PERF_TRACE already defined when loading perf_profil module"
#endif

#define PPRZ_PERF_TRACE(_x) perf_profil_log(_x)
#define PPRZ_PERF_EVENT_START() perf_profil_event_start()
#define PPRZ_PERF_EVENT_END() perf_profil_event_end()

#endif  // PERF_PROFIL_H
