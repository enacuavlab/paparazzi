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
 * @file "modules/sensors/trisonica_mini_LI_550F.c"
 * @author Jean-Baptiste FORESTIER
 * Decoder for standardized messages from the Trisonica Mini LI-550F
 */

#ifndef TRISONICA_MINI_LI_550F_H
#define TRISONICA_MINI_LI_550F_H

#include "std.h"



extern void trisonica_init(void);
extern void trisonica_event(void);
extern void trisonica_report(void);

/**
 * Generic function to send a string command to Jevois
 * @param[in] s string command to send
 */
extern void trisonica_send_string(char *s);

/**
 * Generic function to parse incoming char
 * @param[in] ts structure and c incoming char
 */
extern void trisonica_parse(struct trisonica_t *ts, char c);

/**
 * Generic function to help parsing for uint
 * @param[in] ts structure and c incoming char and the allocation parameter
 */
extern void process_character_uint(struct trisonica_t *ts, char c, int16_t *parameter);

/**
 * Generic function to help parsing for float
 * @param[in] ts structure and c incoming char and the allocation parameter
 */
extern void process_character(struct trisonica_t *ts, char c, float *parameter);

/** Trisonica handle msg
 */
extern void trisonica_handle_msg(struct trisonica_t *ts);

#endif

