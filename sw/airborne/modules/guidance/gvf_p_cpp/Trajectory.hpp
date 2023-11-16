/*
 * Copyright (C) 2023 Mael FEURGARD <mael.feurgard@enac.fr>
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


#pragma once

#include <Eigen/Dense>
#include <string>

#include "TrajEnum.h"


constexpr float DERIVATION_STEP = 1e-6;

class Trajectory
{

public:
  static const TRAJ_ID ID()
  {
    return GENERIC;
  }
  static const std::string Name()
  {
    return "Generic trajectory";
  }

  virtual Eigen::Vector3f pos(float t) const = 0;
  virtual Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f pos_l = pos(t - DERIVATION_STEP);
    Eigen::Vector3f pos_h = pos(t + DERIVATION_STEP);

    return (pos_h-pos_l)/(2*DERIVATION_STEP);
  }

  float speed(float t) const
  {
    return der(t).norm();
  }

  float speed_d(float t) const
  {
    return dder(t).dot(der(t).normalized());
  }

  virtual Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f der_l = der(t - DERIVATION_STEP);
    Eigen::Vector3f der_h = der(t + DERIVATION_STEP);

    return (der_h-der_l)/(2*DERIVATION_STEP);
  }

  float curv_integral(float start, float stop) const;
};
