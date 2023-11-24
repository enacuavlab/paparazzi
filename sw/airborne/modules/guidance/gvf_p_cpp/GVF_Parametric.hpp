// Copyright (C) 2023 Mael FEURGARD <mael.feurgard@enac.fr>
//
// This file is part of paparazzi.
//
// paparazzi is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// paparazzi is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include "Trajectory.hpp"
#include "gvf_adapted_step.h"

typedef struct 
{
  // Position of the carrot
  Eigen::Vector3f target;

  // Direction provided by GVF Parametric
  Eigen::Vector3f direction;

  // Acceleration computed from GVF Parametric
  Eigen::Vector3f acceleration;
} full_output;


class GVF_Parametric
{
  // Trajectory to follow
  Trajectory *traj;

  // Current value for the virtual parameter 'w', and last value of its update 'w_dot'
  float w, w_dot;

  // Diagonal matrix for the k gains
  Eigen::DiagonalMatrix<float,3> k_dmatrix;

  // Additionnal gains (L for error function scaling, beta for trajectory following scaling)
  float L,beta;

  // Time step size (in milliseconds)
  float delta_T;

  // Limit value under witch a float is considered to be null
  const float tolerance = 1e-6;

  // Activate the adaptive stepping method
  bool adapt_step;

  // Compute the values of the GVF Parametric field without normalizing the curve (no adaptive stepping)
  full_output compute_step(float x, float y, float z, float w);

  // Compute the values of the GVF Parametric field with curve normalization (adaptive stepping enabled)
  full_output compute_normalized_step(float x, float y, float z, float w);
};
