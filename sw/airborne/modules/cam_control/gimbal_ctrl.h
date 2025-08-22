/*
 * Copyright (C) 2025 Julia Cabarbaye <julia.cabarbaye1@gmail.com>
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

/** @file "modules/cam_control/gimbal_ctrl.h"
 * @author Julia Cabarbaye <julia.cabarbaye1@gmail.com>
 * caddx gm3 gimbal control sbus
 */

#ifndef GIMBAL_CTRL_H
#define GIMBAL_CTRL_H

#include "stdint.h"

extern void gimbal_ctrl_init(void);
extern void gimbal_ctrl_periodic(void);
extern void gimbal_ctrl_datalink(uint8_t* buf);	
extern int16_t convert_to_CAM_boundary(int8_t channel, int8_t coef);

#endif  // GIMBAL_CTRL_H
