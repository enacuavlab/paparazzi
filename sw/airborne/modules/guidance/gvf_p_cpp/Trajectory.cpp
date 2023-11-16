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

#include "Trajectory.hpp"
#include "trajectories/Addition.hpp"
#include "trajectories/Product.hpp"
#include "trajectories/Affine.hpp"

float Trajectory::curv_integral(float start, float stop) const
{
  
}

AddedTrajectory operator+(const Trajectory& lhs, const Trajectory& rhs)
{
  return AddedTrajectory(lhs,rhs);
}

Translated operator+(const Trajectory& lhs, Eigen::Vector3f rhs)
{
  return Translated(lhs,rhs);
}

Translated operator+(Eigen::Vector3f lhs, const Trajectory& rhs)
{
  return Translated(rhs,lhs);
}

SubstractedTrajectory operator-(const Trajectory& lhs, const Trajectory& rhs)
{
  return SubstractedTrajectory(lhs,rhs);
}

Translated operator-(const Trajectory& lhs, Eigen::Vector3f rhs)
{
  return Translated(lhs,-rhs);
}

Affine operator-(Eigen::Vector3f lhs, const Trajectory& rhs)
{
  return Affine(rhs,-Eigen::Matrix3f::Identity(),lhs);
}

MultipliedTrajectory operator*(const Trajectory& lhs, const Trajectory& rhs)
{
  return MultipliedTrajectory(lhs,rhs);
}

Affine operator*(Eigen::Matrix3f lhs,const Trajectory& rhs)
{
  return Affine(rhs,lhs,Eigen::Vector3f::Zero());
}

Affine operator*(float lhs,const Trajectory& rhs)
{
  return Affine(rhs,lhs*Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero());
}

Affine operator*(const Trajectory& lhs, float rhs)
{
  return Affine(lhs,rhs*Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero());
}

Affine operator*(Eigen::Affine3f lhs,const Trajectory& rhs)
{
  return Affine(rhs,lhs);
}