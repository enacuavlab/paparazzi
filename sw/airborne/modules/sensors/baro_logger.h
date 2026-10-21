/*
 * Copyright (C) 2026 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
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
 *
 */

/**
 * @file modules/sensors/baro_logger.h
 *
 * This reads the values for pressure, temperature and GPS and logs them to the SD card.
 */

#ifndef BARO_LOGGER_H
#define BARO_LOGGER_H


/** Initialize ABI subscriptions and variables. */
extern void tawaki_baro_logger_init(void);

/** Average received samples and periodically write on SD card */
extern void tawaki_baro_logger_periodic(void);

#endif
