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

#include "../Trajectory.hpp"
#include "../TrajEnum.h"

// ---------- Generic affine transform ---------- //

class Affine : public Trajectory
{
private:
  Eigen::Affine3f tr;

protected:
  const Trajectory &base;

public:
  Affine(const Trajectory &traj, Eigen::Affine3f t)
      : base(traj), tr(t) {}

  Affine(const Trajectory &traj, Eigen::Matrix3f linear, Eigen::Vector3f translation)
      : base(traj)
  {
    tr.linear() = linear;
    tr.translation() = translation;
  }

  static const TRAJ_ID ID()
  {
    return AFFINE;
  };

  const std::string Name() const
  {
    return "Affine modified Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return tr * base.pos(t);
  }

  Eigen::Vector3f der(float t) const
  {
    return tr.linear() * base.der(t);
  }

  Eigen::Vector3f dder(float t) const
  {
    return tr.linear() * base.dder(t);
  }
};

// ---------- Translateds ---------- //

class Translated : public Trajectory
{
private:
  Eigen::Vector3f tr;
  const Trajectory &base;

public:
  Translated(const Trajectory &traj, Eigen::Vector3f v)
  : base(traj),tr(v) {}

  Translated(const Trajectory &traj, float tx, float ty, float tz)
  : base(traj),tr(tx,ty,tz) {}

  static const TRAJ_ID ID()
  {
    return TRANSLATION;
  }

  const std::string Name() const
  {
    return "Translated Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return base.pos(t) + tr;
  }

  Eigen::Vector3f der(float t) const
  {
    return base.der(t);
  }

  Eigen::Vector3f dder(float t) const
  {
    return base.dder(t);
  }

};

// ---------- Rotateds ---------- //

class Rotated : public Trajectory
{
private:
  Eigen::Quaternion<float> q;
  const Trajectory &base;

public:
  Rotated(const Trajectory &traj, Eigen::Quaternion<float> r)
      : base(traj), q(r)
  {
    q.normalize();
  }

  Rotated(const Trajectory &traj, Eigen::Matrix3f r)
      : base(traj), q(r)
  {
    q.normalize();
  }

  static const TRAJ_ID ID()
  {
    return ROTATION;
  }

  const std::string Name() const
  {
    return "Rotated Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    return q * base.pos(t);
  }

  Eigen::Vector3f der(float t) const
  {
    return q * base.der(t);
  }

  Eigen::Vector3f dder(float t) const
  {
    return q * base.dder(t);
  }
};

// ---------- Permutation ---------- //

class Permutation : public Trajectory
{
private:
  const int s0, s1, s2;
  const Trajectory &base;

public:
  Permutation(const Trajectory &traj, int i0, int i1, int i2)
      : base(traj), s0(i0), s1(i1), s2(i2) {}

  static const TRAJ_ID ID()
  {
    return PERMUTATION;
  }

  const std::string Name() const
  {
    return "Permuted trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f tmp = base.pos(t);
    Eigen::Vector3f res;
    res[0] = tmp[s0];
    res[1] = tmp[s1];
    res[2] = tmp[s2];
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f tmp = base.der(t);
    Eigen::Vector3f res;
    res[0] = tmp[s0];
    res[1] = tmp[s1];
    res[2] = tmp[s2];
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f tmp = base.dder(t);
    Eigen::Vector3f res;
    res[0] = tmp[s0];
    res[1] = tmp[s1];
    res[2] = tmp[s2];
    return res;
  }
};

// ---------- 1D Projections canonical projections ---------- //

class ProjX : public Trajectory
{
private:
  const Trajectory &base;

public:
  ProjX(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_X;
  }

  const std::string Name() const
  {
    return "X-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[1] = 0.;
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[1] = 0.;
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[1] = 0.;
    res[2] = 0.;
    return res;
  }
};

class ProjY : public Trajectory
{
private:
  const Trajectory &base;

public:
  ProjY(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_Y;
  };

  const std::string Name() const
  {
    return "Y-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[0] = 0.;
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[0] = 0.;
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[0] = 0.;
    res[2] = 0.;
    return res;
  }
};

class ProjZ : public Trajectory
{
private:
  const Trajectory &base;

public:
  ProjZ(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_Z;
  };

  const std::string Name() const
  {
    return "Z-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[1] = 0.;
    res[0] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[1] = 0.;
    res[0] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[1] = 0.;
    res[0] = 0.;
    return res;
  }
};

// ---------- 2D Projections canonical projections ---------- //

class ProjXY : public Trajectory
{
private:
  const Trajectory &base;

public:
  ProjXY(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_XY;
  };

  const std::string Name() const
  {
    return "XY-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[2] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[2] = 0.;
    return res;
  }
};

class ProjXZ : public Trajectory
{

private:
  const Trajectory &base;

public:
  ProjXZ(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_XZ;
  };

  const std::string Name() const
  {
    return "XZ-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[1] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[1] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[1] = 0.;
    return res;
  }
};

class ProjYZ : public Trajectory
{

private:
  const Trajectory &base;

public:
  ProjYZ(const Trajectory &traj) : base(traj) {}

  static const TRAJ_ID ID()
  {
    return PROJ_YZ;
  };

  const std::string Name() const
  {
    return "YZ-Projected Trajectory: [ " + base.Name() + " ]";
  }

  Eigen::Vector3f pos(float t) const
  {
    Eigen::Vector3f res = base.pos(t);
    res[0] = 0.;
    return res;
  }

  Eigen::Vector3f der(float t) const
  {
    Eigen::Vector3f res = base.der(t);
    res[0] = 0.;
    return res;
  }

  Eigen::Vector3f dder(float t) const
  {
    Eigen::Vector3f res = base.dder(t);
    res[0] = 0.;
    return res;
  }
};