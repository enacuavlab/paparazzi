/*
 * Copyright (C) 2023 Mael Feurgard <maelfeurgard@gmail.com>
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

/**
 * @file modules/guidance/gvf_parametric/gvf_adapted_step.c
 *
 * Dynamic parametric step adaptation for the GVF algorithm
 */

#include "gvf_adapted_step.h"

#define ABS(x) (((x) > 0.) ? (x) : -(x))

float p4_eval(float x, float a4, float a3, float a2, float a1, float a0)
{
  return a0 + x * (a1 + x * (a2 + x * (a3 + x * (a4))));
}

float p4_newton(float a4, float a3, float a2, float a1, float a0, float tol, float init)
{
  float x = init;
  float p_x = p4_eval(x, a4, a3, a2, a1, a0);
  float a4_d = 0.;
  float a3_d = 4*a4;
  float a2_d = 3*a3;
  float a1_d = 2*a2;
  float a0_d = a1;

  while (ABS(p_x) > tol)
  {
    float p_xd = p4_eval(x,a4_d,a3_d,a2_d,a1_d,a0_d);
    p_x = p4_eval(x, a4, a3, a2, a1, a0);
    x = x - p_x/p_xd;
  }

  return x;
}

float step_adaptation(float ds, float f1d, float f2d, float f3d, float f1dd, float f2dd, float f3dd)
{
  float a4 = (f1dd * f1dd + f2dd * f2dd + f3dd * f3dd) / 4.;
  float a3 = (f1d * f1dd + f2d * f2dd + f3d * f3dd) / 2.;
  float a2 = (f1d * f1d + f2d * f2d + f3d * f3d);
  float a1 = 0.;
  float a0 = ds * ds;

  if (ds > 0.)
    return p4_newton(a4,a3,a2,a1,a0,1e-6,1.);
  else
    return p4_newton(a4,a3,a2,a1,a0,1e-6,-1.);
}