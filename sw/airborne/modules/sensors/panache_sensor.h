/*
 * Copyright (C) 2023 Fabien-B <fabien.bonneval@enac.fr>
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

/** @file "modules/sensors/panache_sensor.h"
 * @author Fabien-B <fabien.bonneval@enac.fr>
 * Get data from Panache sensors, or datalink (for simulation)
 */

#ifndef PANACHE_SENSOR_H
#define PANACHE_SENSOR_H

#include "stdint.h"

typedef struct {
  struct {
    float  lpl;         //#0
    float  spl;         //#1
    float  mpl;         //#2
    float  pressure;    //#3
    float  tempNtc0;    //#4
    float  tempNtc1;    //#5
    float  tempAduc;    //#6
    float  humidity;    //#7
    float  tempBme280;  //#8
    struct {
      float  irLow;            //#9,13,17
      float  irHigh;           //#10,14,18
      float  irSignal;         //#11,15,19
      float  irSignalFiltered; //#12,16,20
    } channels[3];
  } gas;
  struct {
    float  bin[16];             //#21..36
    float  bin1Mtof;            //#37
    float  bin3Mtof;            //#38
    float  bin5Mtof;            //#39
    float  bin7Mtof;            //#40
    float  sampleFlowRate;      //#41
    float  temperature;         //#42
    float  relativeHumidity;    //#43
    float  samplePeriod;        //#44
    float  rejectCountGlitch;   //#45
    float  rejectCountLong;     //#46
    float  pm[3];               //#47..49
  } particle;
} SensorsData_t;


extern void panache_init(void);
extern void panache_dl_cb(uint8_t* buf);  // PAYLOAD_COMMAND
extern void panache_sensors_cb(uint8_t* buf);  // PAYLOAD_FLOAT

#endif  // PANACHE_SENSOR_H
