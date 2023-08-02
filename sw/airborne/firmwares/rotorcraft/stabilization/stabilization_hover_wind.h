/*
 * Copyright (C) Florian Sansou <florian.sansou@enac.fr>
 *  *
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

#ifndef STABILIZATION_HOVER_WIND
#define STABILIZATION_HOVER_WIND

#include <inttypes.h>



extern void stabilization_hover_wind_init(void);
extern void stabilization_hover_wind_run(bool in_flight);
extern void stabilization_hover_takeoff(void);
extern void stabilization_fill_cmd(void);
extern void parse_wind_info_msg(uint8_t *buf);


#endif /* STABILIZATION_HOVER_WIND */

