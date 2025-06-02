
/*
 * Copyright (C) 2020 Mael Feurgard <maelfeurgard@gmail.com>
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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_drift_ellipse.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * Drifting growing ellipsis (ellipsis on the XY plane, drifting along X)
 * f(t) = [ a_x*t*cos(2*pi*f*t + phi) + t*v_x, a_y*t*sin(2*pi*f*t + phi), 0 ]
 */

#include "modules/nav/common_nav.h"
//#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "../gvf_parametric.h"
#include "gvf_parametric_drift_ellipse.h"

/*! Default gain kx for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KX
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KX 0.01
#endif

/*! Default gain ky for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KY
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KY 0.01
#endif

/*! Default gain kz for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KZ
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KZ 0.01
#endif

/*! Default x-speed for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_V_X
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_V_X 1
#endif

/*! Default x-amplitude for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_X
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_X 1
#endif

/*! Default y-amplitude for the trajectory */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_Y
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_Y 100
#endif

/*! Default frequency */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_FREQ
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_FREQ 1
#endif

/*! Default phase */
#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_PHASE
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_PHASE   0
#endif


gvf_par_drift_ellipse_par gvf_parametric_drift_ellipse_par = {GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KX,
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KY, GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_KZ, 
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_V_X,
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_X,
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_A_Y, 
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_FREQ,
                                                  GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_PHASE};



void gvf_parametric_drift_ellipse_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float v_x = gvf_parametric_trajectory.p_parametric[0];
  float a_x = gvf_parametric_trajectory.p_parametric[1];
  float a_y = gvf_parametric_trajectory.p_parametric[2];
  float freq = gvf_parametric_trajectory.p_parametric[3];
  float phi = gvf_parametric_trajectory.p_parametric[4];

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  float u = 2*M_PI*freq;

  *f1 = a_x*wb*cosf(u*wb + phi) + wb*v_x;
  *f2 =  a_y*wb*sinf(u*wb + phi);
  *f3 = 0.;
  *f1d = v_x + a_x * (cosf(u*wb + phi) - u*wb*sinf(u*wb+phi));
  *f2d = a_y * (sinf(u*wb + phi) + u*wb*cosf(u*wb+phi));
  *f3d = 0.;
  *f1dd = - a_x * u * (2*sinf(u*wb + phi) + u*wb*cosf(u*wb+phi));
  *f2dd = a_y * u * (2*cosf(u*wb + phi) - u*wb*sinf(u*wb+phi));
  *f3dd = 0.;
}