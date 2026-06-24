/*
 * Copyright (C) 2026  ENAC, Mael Feurgard
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/extended_dubins/nav_dubins_gvf.h
 *
 * Extension to the Extended Dubins Paths module using Parametric Guiding Vector Field for guidance instead of usual nav primitives.
 * Junctions between Dubins elements are smoothed out using C2 Hermite interpolation poylnomials
 */

#ifndef NAV_DUBINS_GVF_H
#define NAV_DUBINS_GVF_H

#include "dubins_common.h"

#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "math/pprz_polyfit_float.h"

// bool nav_dubins_gvf_init(DubinsElement_t* elements, int el_num);
void nav_dubins_gvf_track(DubinsElement_t* elements, int el_num, float openloop_w, float nominal_speed);

#endif //NAV_DUBINS_GVF_H