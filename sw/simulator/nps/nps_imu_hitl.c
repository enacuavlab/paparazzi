/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

#include "nps_ins.h"
#include <sys/time.h>
#include "nps_fdm.h"
#include <time.h>
#include <stdio.h>
#include "nps_sensors.h"
#include "nps_ivy.h"
#include "nps_main.h"

void *nps_sensors_loop(void *data __attribute__((unused)));
pthread_t th_sensors; // send sensors over IVY for now

void nps_hitl_impl_init(void)
{
  pthread_create(&th_sensors, NULL, nps_sensors_loop, NULL);
}

void *nps_sensors_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = (1. / PERIODIC_FREQUENCY) * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  while (TRUE) {
    // lock mutex
    pthread_mutex_lock(&fdm_mutex);

    // start timing
    clock_get_current_time(&requestStart);

    nps_ivy_hitl(&sensors);

    // unlock mutex
    pthread_mutex_unlock(&fdm_mutex);

    clock_get_current_time(&requestEnd);

    // Calculate time it took
    task_ns = (requestEnd.tv_sec - requestStart.tv_sec) * 1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    // task took less than one period, sleep for the rest of time
    if (task_ns < period_ns) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = period_ns - task_ns;
      nanosleep(&waitFor, NULL);
    } else {
      // task took longer than the period
#ifdef PRINT_TIME
      printf("SENSORS: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}


