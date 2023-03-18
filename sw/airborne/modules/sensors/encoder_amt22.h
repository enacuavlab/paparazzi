/*
 * Copyright (C) 2023 Flo&Fab <name.surname@enac.fr>
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

/** @file "modules/sensors/encoder_amt22.h"
 * @author Flo&Fab <name.surname@enac.fr>
 * Driver for AMT22 encoder from CUI devices.
 * add <module name="encoder" type="amt22"/>
 */

#ifndef ENCODER_AMT22_H
#define ENCODER_AMT22_H

#include "filters/high_gain_filter.h"
#include "peripherals/amt22.h"

struct EncoderAmt22 {
  struct amt22_t amt22;
  struct high_gain_filter H_g_filter;
};

extern struct EncoderAmt22 encoder_amt22;

extern void encoder_amt22_init(void);
extern void encoder_amt22_periodic(void);
extern void encoder_amt22_event(void);

extern void encoder_amt22_update_alpha0(float alpha0);
extern void encoder_amt22_update_alpha1(float alpha1);
extern void encoder_amt22_update_alpha2(float alpha2);
extern void encoder_amt22_update_epsilon(float epsilon);

extern void encoder_amt22_report(void);

#endif  // ENCODER_AMT22_H
