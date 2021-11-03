/*
 * Copyright (C) 2021 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric_surf/trajectories/gvf_parametric_surf_3d_torus.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D torus figures
 */

#ifndef GVF_PARAMETRIC_SURF_3D_TORUS_H
#define GVF_PARAMETRIC_SURF_3D_TORUS_H

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef gvf_3d_torus_par
* @brief Parameters for the GVF parametric 3D torus
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param rh Horizontal radius
* @param rv Vertical radius
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float rh;
  float rv;
} gvf_par_surf_3d_torus_par;

extern gvf_par_surf_3d_torus_par gvf_parametric_surf_3d_torus_par;

extern void gvf_parametric_surf_3d_torus_info(float *f1, float *f2, float *f3, float *f1dw1, float *f2dw1, float *f3dw1, float *f1ddw1, float *f2ddw1, float *f3ddw1, float *f1dw2, float *f2dw2, float *f3dw2, float *f1ddw2, float *f2ddw2, float *f3ddw2);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_SURF_3D_TORUS_H
