// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <array>

namespace geo {
constexpr double pi = 3.14159265358979323846;
constexpr double rad = pi / 180.;
struct Point {
  double lat = 0, lon = 0, alt = 0;
}; // degrees, metres; ECEF/ENU transformations use ellipsoidal altitude
struct Utm {
  double east, north;
  int zone;
}; // WGS84, signed northings (no southern false northing)
Point fromEcef(std::array<double, 3> ecef);
std::array<double, 3> toEcef(Point point);
Point fromEnu(std::array<double, 3> origin, double east, double north, double up);
Point offset(Point origin, double east, double north, double up = 0);
Point fromUtm(double east, double north, int zone);
Utm toUtm(Point p);
Point advance(Point origin, double bearing, double distance); // radians, metres
double distance(Point from, Point to); // surface distance in metres
double normalize(double radians);
}
