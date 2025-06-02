// Copyright (C) 2023 mael
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

class Sinusoid : Trajectory
{
public:
  float a_x, a_y, a_z;
  float f_x, f_y, f_z;
  float phi_x, phi_y, phi_z;

  Sinusoid(float ax, float ay, float az,
           float fx, float fy, float fz,
           float phix, float phiy, float phiz)
      : a_x(ax), a_y(ay), a_z(az),
        f_x(fx), f_y(fy), f_z(fz),
        phi_x(phix), phi_y(phiy), phi_z(phiz) {}

  Sinusoid(Eigen::Vector3f a,
           Eigen::Vector3f f,
           Eigen::Vector3f phi)
      : a_x(a.x()), a_y(a.y()), a_z(a.z()),
        f_x(f.x()), f_y(f.y()), f_z(f.z()),
        phi_x(phi.x()), phi_y(phi.y()), phi_z(phi.z()) {}

  static const TRAJ_ID ID()
  {
    return SINUSOID;
  }

  static const std::string Name()
  {
    return "3D Sinusoid";
  }

  Eigen::Vector3f pos(float t) const
  {
    float x = a_x * sin(f_x * t + phi_x);
    float y = a_y * sin(f_y * t + phi_y);
    float z = a_z * sin(f_z * t + phi_z);

    return Eigen::Vector3f(x, y, z);
  }

  Eigen::Vector3f der(float t) const
  {
    float x = a_x * f_x * cos(f_x * t + phi_x);
    float y = a_y * f_y * cos(f_y * t + phi_y);
    float z = a_z * f_z * cos(f_z * t + phi_z);

    return Eigen::Vector3f(x, y, z);
  }

  Eigen::Vector3f dder(float t) const
  {
    float x = -a_x * f_x * f_x * sin(f_x * t + phi_x);
    float y = -a_y * f_y * f_y * sin(f_y * t + phi_y);
    float z = -a_z * f_z * f_z * sin(f_z * t + phi_z);

    return Eigen::Vector3f(x, y, z);
  }
};

class EllipseXY : public Sinusoid
{
public:
  EllipseXY(float a, float b, float f, float phi)
  : Sinusoid(a,b,0,f,f,0,phi-M_PI_2, phi, 0) {}

  static const std::string Name()
  {
    return "XY Ellipse";
  }
};

class CircleXY : public EllipseXY
{
public:
  CircleXY(float r, float f, float phi)
  : EllipseXY(r,r,f,phi) {}

  static const std::string Name()
  {
    return "XY Circle";
  }
};

class EllipseXZ : public Sinusoid
{
public:
  EllipseXZ(float a, float b, float f, float phi)
  : Sinusoid(a,0,b,f,0,f,phi-M_PI_2,0, phi) {}

  static const std::string Name()
  {
    return "XZ Ellipse";
  }
};

class CircleXZ : public EllipseXZ
{
public:
  CircleXZ(float r, float f, float phi)
  : EllipseXZ(r,r,f,phi) {}

  static const std::string Name()
  {
    return "XZ Circle";
  }
};

class EllipseYZ : public Sinusoid
{
public:
  EllipseYZ(float a, float b, float f, float phi)
  : Sinusoid(0,a,b,0,f,f,0,phi-M_PI_2, phi) {}

  static const std::string Name()
  {
    return "YZ Ellipse";
  }
};

class CircleYZ : public EllipseYZ
{
public:
  CircleYZ(float r, float f, float phi)
  : EllipseYZ(r,r,f,phi) {}

  static const std::string Name()
  {
    return "YZ Circle";
  }
};