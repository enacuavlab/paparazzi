/*
 * Copyright (C) 2022 Fabien-B <fabien.bonneval [at] gmail [dot] com>
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

/** @file "modules/sensors/thermal_mlx90640.h"
 * @author Fabien-B <fabien.bonneval [at] gmail [dot] com>
 * Module for the MLX90640 themal IR sensor from Melexis.
 */

#ifndef THERMAL_MLX90640_H
#define THERMAL_MLX90640_H

extern void thermal_mlx90640_init(void);
extern void thermal_mlx90640_periodic(void);
extern void thermal_mlx90640_event(void);

#endif  // THERMAL_MLX90640_H
