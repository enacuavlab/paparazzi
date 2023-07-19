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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_growing_lissajou.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * Linear growth Lissajou curve along the x-axis 3D (oscillations along Y and Z axes)
 * f(t) = [ a_x*t, a_y*t*cosf(2*pi*f_y*t + phi_y), a_z*t*sinf(2*pi*f_z*t + phi_z) ]
 */

#ifndef GVF_PARAMETRIC_3D_GROWING_LISSAJOU_H
#define GVF_PARAMETRIC_3D_GROWING_LISSAJOU_H

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef gvf_par_3d_growing_lissajou_par
* @brief Parameters for the GVF parametric 3D linear/log growth Lissajou curve
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param ax Speed along the x-axis
* @param ay Oscillations' amplitude along the y-axis
* @param az Oscillations' amplitude along the z-axis
* @param f_y Frequency along the y-axis (in radiants, i.e. before multiplication by 2*PI)
* @param phi_y Phase for the y-oscillations
* @param f_z Frequency along the z-axis (in radiants, i.e. before multiplication by 2*PI)
* @param phi_z Phase for the z-oscillations
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float ax;
  float ay;
  float az;
  float f_y;
  float phi_y;
  float f_z;
  float phi_z;
} gvf_par_3d_growing_lissajou_par;

extern gvf_par_3d_growing_lissajou_par gvf_parametric_3d_growing_lissajou_par;

extern void gvf_parametric_3d_growing_lissajou_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_3D_GROWING_LISSAJOU_H
