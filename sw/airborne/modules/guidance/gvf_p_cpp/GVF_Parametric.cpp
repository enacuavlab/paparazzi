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

#include "GVF_Parametric.hpp"

full_output GVF_Parametric::compute_step(float x, float y, float z, float w)
{
  Eigen::Vector3f traj_pos = traj->pos(beta * w);
  Eigen::Vector3f traj_vel = traj->der(beta * w);
  Eigen::Vector3f traj_acc = traj->dder(beta * w);

  Eigen::Vector3f pos(x, y, z);
  Eigen::Vector3f phi = L * (pos - traj_pos);

  Eigen::Vector3f chi_phys = -traj_vel * L * L * beta - k_dmatrix * phi;
  float chi_virt = -L * L + beta * (traj_vel.dot(k_dmatrix * phi));
  chi_phys *= L;
  chi_virt *= L;

  full_output res;
  res.target.x() = traj_pos.x();
  res.target.y() = traj_pos.y();
  res.target.z() = traj_pos.z();
  res.target.w() = w;

  res.direction.x() = chi_phys.x();
  res.direction.y() = chi_phys.y();
  res.direction.z() = chi_phys.z();
  res.direction.w() = chi_virt;

  res.jac.setZero();
  res.jac(0, 0) = -k_dmatrix.diagonal()(0) * L;
  res.jac(1, 1) = -k_dmatrix.diagonal()(1) * L;
  res.jac(2, 2) = -k_dmatrix.diagonal()(2) * L;
  res.jac(3, 0) = k_dmatrix.diagonal()(0) * traj_vel(0) * beta * L;
  res.jac(3, 1) = k_dmatrix.diagonal()(1) * traj_vel(1) * beta * L;
  res.jac(3, 2) = k_dmatrix.diagonal()(2) * traj_vel(2) * beta * L;
  res.jac(0, 3) = -(beta * L) * (beta * L * traj_acc(0) - k_dmatrix.diagonal()(0) * traj_vel(0));
  res.jac(1, 3) = -(beta * L) * (beta * L * traj_acc(1) - k_dmatrix.diagonal()(1) * traj_vel(1));
  res.jac(2, 3) = -(beta * L) * (beta * L * traj_acc(2) - k_dmatrix.diagonal()(2) * traj_vel(2));
  res.jac(3, 3) = beta * beta * (traj_acc.dot(k_dmatrix*phi) - L * traj_vel.dot(k_dmatrix*traj_vel));
  res.jac *= L;

  return res;
}

full_output GVF_Parametric::compute_normalized_step(float x, float y, float z, float w)
{
  Eigen::Vector3f traj_pos = traj->pos(beta * w);
  Eigen::Vector3f traj_vel = traj->der(beta * w);
  Eigen::Vector3f traj_acc = traj->dder(beta * w);

  float traj_speed_squared = traj_vel.dot(traj_vel);
  float traj_speed = sqrtf(traj_speed_squared);

  Eigen::Vector3f traj_norm_vel;
  Eigen::Vector3f traj_norm_acc;
  if (traj_speed < tolerance)
  {
    // Consider speed to be zero
    // Use arbitrary direction and null acceleration
    traj_norm_vel = Eigen::Vector3f::Ones() / sqrtf(3);
    traj_norm_acc = Eigen::Vector3f::Zero();
  }
  else
  {
    traj_norm_vel = traj_vel / traj_speed;
    traj_norm_acc = (traj_acc - traj_vel.dot(traj_acc) * traj_vel / traj_speed_squared) / traj_speed_squared;
  }

  Eigen::Vector3f pos(x, y, z);
  Eigen::Vector3f phi = L * (pos - traj_pos);

  Eigen::Vector3f chi_phys = -traj_norm_vel * L * L * beta - k_dmatrix * phi;
  float chi_virt = -L * L + beta * (traj_norm_vel.dot(k_dmatrix * phi));
  chi_phys *= L;
  chi_virt *= L;
  chi_virt = step_adaptation(ground_speed * chi_virt * delta_T,
                             traj_norm_vel.x(), traj_norm_vel.y(), traj_norm_vel.z(),
                             traj_norm_acc.x(), traj_norm_acc.y(), traj_norm_acc.z()) /
             delta_T;

  full_output res;
  res.target.x() = traj_pos.x();
  res.target.y() = traj_pos.y();
  res.target.z() = traj_pos.z();
  res.target.w() = w;

  res.direction.x() = chi_phys.x();
  res.direction.y() = chi_phys.y();
  res.direction.z() = chi_phys.z();
  res.direction.w() = chi_virt;

  res.jac.setZero();
  res.jac(0, 0) = -k_dmatrix.diagonal()(0) * L;
  res.jac(1, 1) = -k_dmatrix.diagonal()(1) * L;
  res.jac(2, 2) = -k_dmatrix.diagonal()(2) * L;
  res.jac(3, 0) = k_dmatrix.diagonal()(0) * traj_norm_vel(0) * beta * L;
  res.jac(3, 1) = k_dmatrix.diagonal()(1) * traj_norm_vel(1) * beta * L;
  res.jac(3, 2) = k_dmatrix.diagonal()(2) * traj_norm_vel(2) * beta * L;
  res.jac(0, 3) = -(beta * L) * (beta * L * traj_norm_acc(0) - k_dmatrix.diagonal()(0) * traj_norm_vel(0));
  res.jac(1, 3) = -(beta * L) * (beta * L * traj_norm_acc(1) - k_dmatrix.diagonal()(1) * traj_norm_vel(1));
  res.jac(2, 3) = -(beta * L) * (beta * L * traj_norm_acc(2) - k_dmatrix.diagonal()(2) * traj_norm_vel(2));
  res.jac(3, 3) = beta * beta * (traj_norm_acc.dot(k_dmatrix*phi) - L * traj_norm_vel.dot(k_dmatrix*traj_norm_vel));
  res.jac *= L;

  return res;
}