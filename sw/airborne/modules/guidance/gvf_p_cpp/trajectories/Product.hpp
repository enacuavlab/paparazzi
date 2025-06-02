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

class MultipliedTrajectory : public Trajectory
{
private:
  const Trajectory& left;
  const Trajectory& right;

public:
  MultipliedTrajectory(const Trajectory& l_traj, const Trajectory& r_traj)
      : left(l_traj), right(r_traj) {}

  static const TRAJ_ID ID()
  {
    return PRODUCT;
  };

  const std::string Name() const
  {
    return "Multiplied Trajectories: [ " + left.Name() + " ] * [ " + right.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return left.pos(t).array() * right.pos(t).array();
  }

  Eigen::Vector3f der(float t) const
  {
    return left.der(t).array() * right.pos(t).array() + left.pos(t).array() * right.der(t).array();
  }

  Eigen::Vector3f dder(float t) const
  {
    return left.dder(t).array() * right.pos(t).array() + 2 * left.der(t).array() * right.der(t).array() + left.pos(t).array() * right.dder(t).array();
  }
};