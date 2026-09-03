/*
 * Copyright (C) 2026 Fabien-B <fabien-B@github.com>
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

/** @file "modules/imav_rescue_people/scoutector.h"
 * @author Fabien-B <fabien-B@github.com>
 * Scoutector driver, specially made for IMAV 2026 !
 */

#ifndef SCOUTECTOR_H
#define SCOUTECTOR_H


typedef struct {
  float det;
  float snr;
  float lit;
} scoutector_t;

extern void scoutector_init(void);
extern void scoutector_report(void);

extern scoutector_t scout_data;

#endif  // SCOUTECTOR_H
