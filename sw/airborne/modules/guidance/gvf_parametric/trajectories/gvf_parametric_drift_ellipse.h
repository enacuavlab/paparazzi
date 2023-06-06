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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_drift_ellipse.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * Drifting growing ellipsis (ellipsis on the XY plane, drifting along X)
 * f(t) = [ -a_y*t*sin(2*pi*f*log(abs(t) + 1) + phi) + t*v_x, a_x*t*cos(2*pi*f*log(abs(t) + 1) + phi), 0 ]
 */

#ifndef GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_H
#define GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_H

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef gvf_par_drift_ellipse_par
* @brief Parameters for the GVF parametric 3D linear/log growth Lissajou curve
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param v_x Speed along the x-axis
* @param a_x Oscillations' amplitude along the x-axis
* @param a_y Oscillations' amplitude along the y-axis
* @param freq Frequency (in radiants, i.e. before multiplication by 2*PI)
* @param phi Phase 
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float v_x;
  float a_x;
  float a_y;
  float freq;
  float phi;
} gvf_par_drift_ellipse_par;

extern gvf_par_drift_ellipse_par gvf_parametric_drift_ellipse_par;

extern void gvf_parametric_drift_ellipse_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_3D_DRIFT_ELLIPSE_H
