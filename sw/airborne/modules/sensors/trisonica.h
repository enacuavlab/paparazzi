/*
 * Copyright (C) 2025 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
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
/**
 * @file "modules/sensors/trisonica.c"
 * @author Jean-Baptiste FORESTIER
 * Decoder for standardized messages from the Trisonica Mini LI-550F
 * Calibration Anemo, compas, and Level has to be done with an operator
 * In CLI mode, set your output rate : (ctrl + C, ASCII 3), then , outputrate your_freq
 * In CLI mode, select the parameters you want 
 */

#ifndef TRISONICA_H
#define TRISONICA_H

#include "std.h"



extern void trisonica_init(void);
extern void trisonica_event(void);
extern void trisonica_report(void);
extern void trisonica_handle_msg(struct trisonica_t *ts);
extern void trisonica_send_string(char *s);



#endif

