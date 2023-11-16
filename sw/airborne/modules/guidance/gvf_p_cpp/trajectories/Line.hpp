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

#include "../Trajectory.hpp"
#include "../TrajEnum.h"

class Line : Trajectory
{
public:
  Eigen::Vector3f a,b;

  Line(float ax, float ay, float az,
       float bx, float by, float bz)
      : a(ax,ay,az),
        b(bx,by,bz) {}

  Line(Eigen::Vector3f a, Eigen::Vector3f b)
      : a(a), b(b) {}

  static const TRAJ_ID ID() 
  {
    return LINE;
  }

  static const std::string Name() 
  {
    return "3D Line";
  }

  Eigen::Vector3f pos(float t) const
  {
    return a * t + b;
  }

  Eigen::Vector3f der(float t) const
  {
    return a;
  }

  Eigen::Vector3f dder(float t) const
  {
    return Eigen::Vector3f(0, 0, 0);
  }
};