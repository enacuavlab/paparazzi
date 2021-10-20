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
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "gvf_parametric_3d_torus.h"

/*! Default gain kx for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_3D_TORUS_KX
#define GVF_PARAMETRIC_3D_TORUS_KX 0.001
#endif

/*! Default gain ky for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_3D_TORUS_KY
#define GVF_PARAMETRIC_3D_TORUS_KY 0.001
#endif

/*! Default gain kz for the 3d torus trajectory */
#ifndef GVF_PARAMETRIC_3D_TORUS_KZ
#define GVF_PARAMETRIC_3D_TORUS_KZ 0.001
#endif

/*! Default radius taken from the center of the torus  */
#ifndef GVF_PARAMETRIC_3D_TORUS_RH
#define GVF_PARAMETRIC_3D_TORUS_RH 80
#endif

/*! Default radius for the inner tube */
#ifndef GVF_PARAMETRIC_3D_TORUS_RV
#define GVF_PARAMETRIC_3D_TORUS_RV 10
#endif

gvf_par_3d_torus_par gvf_parametric_3d_torus_par = {GVF_PARAMETRIC_3D_TORUS_KX, GVF_PARAMETRIC_3D_TORUS_KY, GVF_PARAMETRIC_3D_TORUS_KZ, GVF_PARAMETRIC_3D_TORUS_RH, GVF_PARAMETRIC_3D_TORUS_RV};

void gvf_parametric_3d_torus_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float zo = gvf_parametric_trajectory.p_parametric[2];
  float rh = gvf_parametric_trajectory.p_parametric[3];
  float rv = gvf_parametric_trajectory.p_parametric[4];

  float w1 = gvf_parametric_surface_control.w1;
  float w2 = gvf_parametric_surface_control.w2;
  float w1b = w1 * gvf_parametric_surface_control.beta1 * gvf_parametric_surface_control.s1;
  float w2b = w2 * gvf_parametric_surface_control.beta2 * gvf_parametric_surface_control.s2;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'

  *f1 = (rh + rv*cosf(w2))*cosf(w) + xo;
  *f2 = (rh + rv*sinf(w2))*sinf(w) + yo;
  *f3 = rv*sinf(w2) + zo;

  float nrf1d = -wx*cx*sinf(wx*wb + deltax_rad);
  float nrf2d = -wy*cy*sinf(wy*wb + deltay_rad);

  *f1d = cosf(alpha_rad)*nrf1d - sinf(alpha_rad)*nrf2d;
  *f2d = sinf(alpha_rad)*nrf1d + cosf(alpha_rad)*nrf2d;
  *f3d = -wz*cz*sinf(wz*wb + deltaz_rad);

  float nrf1dd = -wx*wx*cx*cosf(wx*wb + deltax_rad);
  float nrf2dd = -wy*wy*cy*cosf(wy*wb + deltay_rad);

  *f1dd = cosf(alpha_rad)*nrf1dd - sinf(alpha_rad)*nrf2dd;
  *f2dd = sinf(alpha_rad)*nrf1dd + cosf(alpha_rad)*nrf2dd;
  *f3dd = -wz*wz*cz*cosf(wz*wb + deltaz_rad);
}

