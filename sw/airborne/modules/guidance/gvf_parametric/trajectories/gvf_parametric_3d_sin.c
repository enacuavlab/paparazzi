/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_sin.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D sinusoid (oscillations along Y and Z axes)
 * f(t) = [ t ,  ay * sinf(2pi * freq_y * t + phase_y) , az * sinf(2pi * freq_z * t + phase_z) ]
 */

#include "modules/nav/common_nav.h"
//#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "../gvf_parametric.h"
#include "gvf_parametric_3d_sin.h"

/*! Default gain kx for the 3d sin trajectory */
#ifndef GVF_PARAMETRIC_3D_SIN_KX
#define GVF_PARAMETRIC_3D_SIN_KX 0.001
#endif

/*! Default gain ky for the 3d sin trajectory */
#ifndef GVF_PARAMETRIC_3D_SIN_KY
#define GVF_PARAMETRIC_3D_SIN_KY 0.001
#endif

/*! Default gain kz for the 3d sin trajectory */
#ifndef GVF_PARAMETRIC_3D_SIN_KZ
#define GVF_PARAMETRIC_3D_SIN_KZ 0.001
#endif

/*! Default amplitude along y-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_AY
#define GVF_PARAMETRIC_3D_SIN_AY 1
#endif


/*! Default phase along y-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_PHASE_Y
#define GVF_PARAMETRIC_3D_SIN_PHASE_Y 0
#endif

/*! Default frequency along z-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_FREQ_Y
#define GVF_PARAMETRIC_3D_SIN_FREQ_Y 1
#endif

/*! Default amplitude along y-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_AZ
#define GVF_PARAMETRIC_3D_SIN_AZ 1
#endif

/*! Default frequency along z-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_FREQ_Z
#define GVF_PARAMETRIC_3D_SIN_FREQ_Z 1
#endif

/*! Default phase along z-axis */
#ifndef GVF_PARAMETRIC_3D_SIN_PHASE_Z
#define GVF_PARAMETRIC_3D_SIN_PHASE_Z 0
#endif

gvf_par_3d_sin_par gvf_parametric_3d_sin_par = {GVF_PARAMETRIC_3D_SIN_KX,
                                                  GVF_PARAMETRIC_3D_SIN_KY, GVF_PARAMETRIC_3D_SIN_KZ, 
                                                  GVF_PARAMETRIC_3D_SIN_AY, GVF_PARAMETRIC_3D_SIN_FREQ_Y,
                                                  GVF_PARAMETRIC_3D_SIN_PHASE_Y,
                                                  GVF_PARAMETRIC_3D_SIN_AZ, GVF_PARAMETRIC_3D_SIN_FREQ_Z,
                                                  GVF_PARAMETRIC_3D_SIN_PHASE_Z};

void gvf_parametric_3d_sin_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float ay = gvf_parametric_trajectory.p_parametric[0];
  float f_y = gvf_parametric_trajectory.p_parametric[1] * 2 * M_PI;
  float phi_y = gvf_parametric_trajectory.p_parametric[2];
  float az = gvf_parametric_trajectory.p_parametric[3];
  float f_z = gvf_parametric_trajectory.p_parametric[4] * 2 * M_PI;
  float phi_z = gvf_parametric_trajectory.p_parametric[5];

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'
  *f1 = wb;
  *f2 = ay * sinf(f_y * wb + phi_y);
  *f3 = az * sinf(f_z * wb + phi_z);

  *f1d = 1;
  *f2d = ay * f_y * cosf(f_y * wb + phi_y);
  *f3d = az * f_z * cosf(f_z * wb + phi_z);

  *f1dd = 0;
  *f2dd = - ay * f_y * f_y * sinf(f_y * wb + phi_y);
  *f3dd = - az * f_z * f_z * sinf(f_z * wb + phi_z);
}

