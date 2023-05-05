/*
 * Copyright (C) 2023 Mael Feurgard <maeL.feurgard@laas.fr>
 *
 * This file is part of paparazzi
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

/** @file "modules/guidance/gvf_mission/gvf_mission.c"
 * @author Mael Feurgard <maeL.feurgard@laas.fr>
 * A wrapper module to allow the use of GVF parametric trajectories in Mission mode.
 */

#include "modules/guidance/gvf_mission/gvf_mission.h"

bool gvf_mission_set_control(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  gvf_parametric_control.w = params[0];
  if (nb < 1)
    return false;

  gvf_parametric_control.delta_T = params[1];
  if (nb < 2)
    return false;

  gvf_parametric_control.s = params[2];
  if (nb < 3)
    return false;

  gvf_parametric_control.k_roll = params[3];
  if (nb < 4)
    return false;

  gvf_parametric_control.k_climb = params[4];
  if (nb < 5)
    return false;

  gvf_parametric_control.k_psi = params[5];
  if (nb < 6)
    return false;

  gvf_parametric_control.L = params[6];
  if (nb < 7)
    return false;

  gvf_parametric_control.beta = params[7];
  if (nb < 8)
    return false;

  gvf_parametric_control.w_dot = params[8];

  return false;
}

bool gvf_mission_set_params(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  for (unit_t i = 0; i < nb; i++)
  {
    gvf_parametric_trajectory.p_parametric[i] = params[i];
  }

  return false;
}

bool gvf_mission_set_transform(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  gvf_parametric_set_affine_tr(params[0], params[1], params[2], params[3], params[4], params[5]);

  return false;
}

bool gvf_mission_set_q_transform(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  gvf_parametric_set_affine_q_tr(params[0], params[1], params[2], params[3], params[4], params[5], params[6]);

  return false;
}

bool gvf_mission_set_trajectory(UNUSED uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  gvf_parametric_trajectory.type = params[0];

  return false;
}

bool gvf_mission_run_trajectory(uint8_t nb, float *params, UNUSED enum MissionRunFlag flag)
{
  static float w_limit = NAN;

  if (nb > 1)
  {
    w_limit = params[1];
  }

  if (nb > 0)
  {
    gvf_parametric_trajectory.type = params[0];
  }

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  switch (gvf_parametric_trajectory.type)
  {
  case TREFOIL_2D:
    gvf_parametric_2d_trefoil_info(&f1, &f2, &f1d, &f2d, &f1dd, &f2dd);
    gvf_parametric_control_2d(gvf_parametric_2d_trefoil_par.kx,
                              gvf_parametric_2d_trefoil_par.ky,
                              f1, f2, f1d, f2d, f1dd, f2dd);
    break;

  case ELLIPSE_3D:
    gvf_parametric_3d_ellipse_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
    gvf_parametric_control_3d(gvf_parametric_3d_ellipse_par.kx,
                              gvf_parametric_3d_ellipse_par.ky,
                              gvf_parametric_3d_ellipse_par.kz,
                              f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

    break;

  case LISSAJOUS_3D:
    gvf_parametric_3d_lissajous_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
    gvf_parametric_control_3d(gvf_parametric_3d_lissajous_par.kx,
                              gvf_parametric_3d_lissajous_par.ky,
                              gvf_parametric_3d_lissajous_par.kz,
                              f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

    break;

  case TORUS_3D_SURFACE:
    // Defined in gvf_parametric.h, but isn't really a trajectory...
    // Do nothing and return
    return false;
    break;

  case SINUS_3D:
    gvf_parametric_3d_sin_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
    gvf_parametric_control_3d(gvf_parametric_3d_sin_par.kx,
                              gvf_parametric_3d_sin_par.ky,
                              gvf_parametric_3d_sin_par.kz,
                              f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

    break;

  default:
    // In doubt, do nothing and stop
    return false;
    break;
  }

  if (isnan(w_limit))
  {
    return true;
  }
  else
  {
    if (ABS(gvf_parametric_control.w) > w_limit)
    {
      w_limit = NAN;
      return false;
    }
    else
    {
      return true;
    }
  }
}

void gvf_mission_register(void)
{
  if(!mission_register(gvf_mission_set_control,"GVFSC"))
  {
    printf("Error while registering function 'gvf_mission_set_control' with code 'GVFSC'\n");
  }
  if(!mission_register(gvf_mission_set_params,"GVFSP"))
  {
    printf("Error while registering function 'gvf_mission_set_params' with code 'GVFSP'\n");
  }
  if(!mission_register(gvf_mission_set_transform,"GVFST"))
  {
    printf("Error while registering function 'gvf_mission_set_transform' with code 'GVFST'\n");
  }
  if(!mission_register(gvf_mission_set_q_transform,"GVFSQ"))
  {
    printf("Error while registering function 'gvf_mission_set_q_transform' with code 'GVFSQ'\n");
  }
  if(!mission_register(gvf_mission_set_trajectory,"GVFTR"))
  {
    printf("Error while registering function 'gvf_mission_set_trajectory' with code 'GVFTR'\n");
  }
  if(!mission_register(gvf_mission_run_trajectory,"GVFGO"))
  {
    printf("Error while registering function 'gvf_mission_run_trajectory' with code 'GVFGO'\n");
  }
}
