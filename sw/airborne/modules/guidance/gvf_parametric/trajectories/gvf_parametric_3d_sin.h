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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_sin.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D sinusoid (oscillations along Y and Z axes)
 * f(t) = [ t ,  ay * sin(2pi * freq_y * t) , az * sin(2pi * freq_z * t) ]
 */

#ifndef GVF_PARAMETRIC_3D_SIN_H
#define GVF_PARAMETRIC_3D_SIN_H

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef gvf_3d_sin_par
* @brief Parameters for the GVF parametric 3D sinusoid
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param ay Oscillations' amplitude along the y-axis
* @param freq_y Frequency along the y-axis
* @param az Oscillations' amplitude along the z-axis
* @param freq_z Frequency along the z-axis
* @param phase Added phase to the z oscillations (relative to y)
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float ay;
  float freq_y;
  float az;
  float freq_z;
  float phase;
} gvf_par_3d_sin_par;

extern gvf_par_3d_sin_par gvf_parametric_3d_sin_par;

extern void gvf_parametric_3d_sin_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_3D_SIN_H
