/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file "modules/ctrl/eff_scheduling_colibri.h"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * Interpolation of control effectivenss matrix of the Colibri.
 */

#ifndef EFF_SCHEDULING_COLIBRI_H
#define EFF_SCHEDULING_COLIBRI_H

extern void ctrl_eff_scheduling_init(void);
extern void ctrl_eff_scheduling_periodic(void);

#endif  // EFF_SCHEDULING_COLIBRI_H
