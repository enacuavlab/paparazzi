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

class AddedTrajectory: public Trajectory
{
private:
  const Trajectory& left;
  const Trajectory& right;

public:
  AddedTrajectory(const Trajectory& l_traj, const Trajectory& r_traj)
      : left(l_traj), right(r_traj) {}

  static const TRAJ_ID ID()
  {
    return SUM;
  };

  const std::string Name() const
  {
    return "Added Trajectories: [ " + left.Name() + " ] + [ " + right.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return left.pos(t) + right.pos(t);
  }

  Eigen::Vector3f der(float t) const
  {
    return left.der(t) + right.der(t);
  }

  Eigen::Vector3f dder(float t) const
  {
    return left.dder(t) + right.dder(t);
  }
};
class SubstractedTrajectory : public Trajectory
{
private:
  const Trajectory& left;
  const Trajectory& right;

public:
  SubstractedTrajectory(const Trajectory& l_traj, const Trajectory& r_traj)
      : left(l_traj), right(r_traj) {}

  static const TRAJ_ID ID()
  {
    return DIFFERENCE;
  };

  const std::string Name() const
  {
    return "Substracted Trajectories: [ " + left.Name() + " ] - [ " + right.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return left.pos(t) - right.pos(t);
  }

  Eigen::Vector3f der(float t) const
  {
    return left.der(t) - right.der(t);
  }

  Eigen::Vector3f dder(float t) const
  {
    return left.dder(t) - right.dder(t);
  }
};