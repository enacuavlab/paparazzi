/*
 * Copyright (C) 2025 Ramon Revilla Bouso <ramonrevilla21@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef GUIDANCE_INDI_HYBRID_TILTROTOR
#define GUIDANCE_INDI_HYBRID_TILTROTOR

#define GIHT_X 0
#define GIHT_Y 1
#define GIHT_Z 2
#define GIHT_CMD_ROLL   0
#define GIHT_CMD_PITCH  1
#define GIHT_CMD_FZ     2
#define GIHT_CMD_FX     3

extern void guidance_indi_tiltrotor_init(void);
extern void guidance_indi_tiltrotor_propagate_filters(void);

extern float gi_pitch_eff_scaling;

#endif // GUIDANCE_INDI_HYBRID_TILTROTOR

