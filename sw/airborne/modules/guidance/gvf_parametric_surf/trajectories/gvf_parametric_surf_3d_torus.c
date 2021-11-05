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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_torus.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D torus
 */

#include "subsystems/navigation/common_nav.h"
#include "modules/guidance/gvf_parametric_surf/gvf_parametric_surf.h"
#include "gvf_parametric_surf_3d_torus.h"

/*! Default gain kx for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_KX
#define GVF_PARAMETRIC_SURF_3D_TORUS_KX 0.001
#endif

/*! Default gain ky for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_KY
#define GVF_PARAMETRIC_SURF_3D_TORUS_KY 0.001
#endif

/*! Default gain kz for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_KZ
#define GVF_PARAMETRIC_SURF_3D_TORUS_KZ 0.001
#endif

/*! Default radius taken from the center of the torus  */
#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_RH
#define GVF_PARAMETRIC_SURF_3D_TORUS_RH 80
#endif

/*! Default radius for the inner tube */
#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_RV
#define GVF_PARAMETRIC_SURF_3D_TORUS_RV 10
#endif

gvf_par_surf_3d_torus_par gvf_parametric_surf_3d_torus_par = {GVF_PARAMETRIC_SURF_3D_TORUS_KX, GVF_PARAMETRIC_SURF_3D_TORUS_KY, GVF_PARAMETRIC_SURF_3D_TORUS_KZ, GVF_PARAMETRIC_SURF_3D_TORUS_RH, GVF_PARAMETRIC_SURF_3D_TORUS_RV};

void gvf_parametric_surf_3d_torus_info(float *f1, float *f2, float *f3, float *f1dw1, float *f2dw1, float *f3dw1, float *f1ddw1, float *f2ddw1, float *f3ddw1, float *f1dw2, float *f2dw2, float *f3dw2, float *f1ddw2, float *f2ddw2, float *f3ddw2)
{
  float xo = gvf_parametric_surf_trajectory.p_parametric[0];
  float yo = gvf_parametric_surf_trajectory.p_parametric[1];
  float zo = gvf_parametric_surf_trajectory.p_parametric[2];
  float rh = gvf_parametric_surf_trajectory.p_parametric[3];
  float rv = gvf_parametric_surf_trajectory.p_parametric[4];

  float w1 = gvf_parametric_surf_control.w1;
  float w2 = gvf_parametric_surf_control.w2;
  float w1b = w1 * gvf_parametric_surf_control.beta1 * gvf_parametric_surf_control.s1;
  float w2b = w2 * gvf_parametric_surf_control.beta2 * gvf_parametric_surf_control.s2;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w1b' and 'w2b'

  *f1 = (rh + rv*cosf(w2b))*cosf(w1b) + xo;
  *f2 = (rh + rv*cosf(w2b))*sinf(w1b) + yo;
  *f3 = rv*sinf(w2b) + zo;

  *f1dw1 = -sinf(w1b)*(rh + rv*cos(w2b));
  *f2dw1 =  cosf(w1b)*(rh + rv*cos(w2b));
  *f3dw1 = 0;

  *f1ddw1 = -cosf(w1b)*(rh + rv*cos(w2b));
  *f2ddw1 = -sinf(w1b)*(rh + rv*cos(w2b));
  *f3ddw1 = 0;

  *f1dw2 = -rv*sinf(w2b)*cosf(w1b);
  *f2dw2 = -rv*sinf(w2b)*sinf(w1b);
  *f3dw2 =  rv*cosf(w2b);

  *f1ddw2 = -rv*cosf(w2b)*cosf(w1b);
  *f2ddw2 = -rv*cosf(w2b)*sinf(w1b);
  *f3ddw2 = -rv*sinf(w2b);

}

