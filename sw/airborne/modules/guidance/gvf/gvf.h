/*
 * Copyright (C) 2016  Hector Garcia de Marina
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
 *
 */

/** \file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_H
#define GVF_H

#define GVF_GRAVITY 9.806

#include "std.h"

// Control
extern float gvf_error;
extern float gvf_ke;
extern float gvf_kn;
extern float gvf_kd;

// Trajectory
extern uint8_t gvf_traj_type;
extern float gvf_p1;
extern float gvf_p2;
extern float gvf_p3;
extern float gvf_p4;
extern float gvf_p5;
extern float gvf_p6;
extern float gvf_p7;

extern void gvf_init(void);
extern bool gvf_ellipse(uint8_t, float, float, float);
extern bool gvf_ellipse_set(uint8_t);

#endif // GVF_H
