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

/** \file gvf_ellipse.c
 *
 *  Guidance algorithm based on vector fields
 *  2D Ellipse trajectory
 */

#ifndef GVF_ELLIPSE_A
#define GVF_ELLIPSE_A 80
#endif

#ifndef GVF_ELLIPSE_B
#define GVF_ELLIPSE_B 80
#endif

#ifndef GVF_ELLIPSE_ALPHA
#define GVF_ELLIPSE_ALPHA 0
#endif

float gvf_ellipse_a = GVF_ELLIPSE_A;
float gvf_ellipse_b = GVF_ELLIPSE_B;
float gvf_ellipse_alpha = GVF_ELLIPSE_ALPHA;
