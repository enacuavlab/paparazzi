
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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_growing_lissajou.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * Linear growth Lissajou curve along the x-axis 3D (oscillations along Y and Z axes)
 * f(t) = [ a_x*t, a_y*t*cosf(2*pi*f_y*t + phi_y), a_z*t*sinf(2*pi*f_z*t + phi_z) ]
 */

#include "modules/nav/common_nav.h"
//#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "../gvf_parametric.h"
#include "gvf_parametric_3d_growing_lissajou.h"

/*! Default gain kx for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KX
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KX 0.01
#endif

/*! Default gain ky for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KY
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KY 0.01
#endif

/*! Default gain kz for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KZ
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KZ 0.01
#endif

/*! Default x-speed for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AX
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AX 1
#endif

/*! Default y-amplitude for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AY
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AY 100
#endif

/*! Default z-amplitude for the trajectory */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AZ
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AZ 1
#endif

/*! Default y-frequency */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Y
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Y 1
#endif

/*! Default y-phase */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Y
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Y   0
#endif


/*! Default z-frequency */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Z
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Z   0
#endif

/*! Default z-phase */
#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Z
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Z   0
#endif

gvf_par_3d_growing_lissajou_par gvf_parametric_3d_growing_lissajou_par = {GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KX,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KY, GVF_PARAMETRIC_3D_GROWING_LISSAJOU_KZ, 
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AX,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AY,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_AZ, 
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Y,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Y,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_F_Z,
                                                  GVF_PARAMETRIC_3D_GROWING_LISSAJOU_PHASE_Z};


void gvf_parametric_3d_growing_lissajou_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float ax = gvf_parametric_trajectory.p_parametric[0];
  float ay = gvf_parametric_trajectory.p_parametric[1];
  float az = gvf_parametric_trajectory.p_parametric[2];
  float f_y = gvf_parametric_trajectory.p_parametric[3];
  float phi_y = gvf_parametric_trajectory.p_parametric[4];
  float f_z = gvf_parametric_trajectory.p_parametric[5];
  float phi_z = gvf_parametric_trajectory.p_parametric[6];

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  float u_y = 2*M_PI * f_y;
  float u_z = 2*M_PI * f_z;

  *f1 = ax * wb;
  *f2 = ay * wb * cosf(u_y*wb + phi_y);
  *f3 = az * wb * sinf(u_z*wb + phi_z);
  *f1d = ax;
  *f2d = ay * cosf(u_y*wb + phi_y) - ay * u_y * wb * sinf(u_y*wb + phi_y);
  *f3d = az * sinf(u_z*wb + phi_z) + az * u_z * wb * cosf(u_z*wb + phi_z);
  *f1dd = 0.;
  *f2dd = - ay * u_y * (2*sinf(u_y*wb+phi_y) + wb * u_y *cosf(u_y*wb+phi_y));
  *f3dd = az * u_z * (2*cosf(u_z*wb+phi_z) - wb * u_z *sinf(u_z*wb+phi_z));
}