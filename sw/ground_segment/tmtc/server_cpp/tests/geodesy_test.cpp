// SPDX-License-Identifier: GPL-2.0-or-later
#include "geodesy.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void near(double actual, double expected, double tolerance) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
    throw std::runtime_error("Expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}

template<class Function>
void rejects(Function operation) {
  try {
    operation();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error("Invalid coordinate was accepted");
}
}

int main() {
  try {
    // Reference examples from PROJ's UTM and topocentric documentation:
    // https://proj.org/en/stable/operations/projections/utm.html
    // https://proj.org/en/stable/operations/conversions/topocentric.html
    const auto northern = geo::fromUtm(687071.44, 6210141.33, 32);
    near(northern.lat, 56, 1e-7);
    near(northern.lon, 12, 1e-7);
    const auto southern = geo::fromUtm(740526.32, 5123750.87 - 10000000, 59);
    near(southern.lat, -44, 1e-7);
    near(southern.lon, 174, 1e-7);
    const auto centralMeridian = geo::toUtm({0, 9, 100});
    if (centralMeridian.zone != 32) throw std::runtime_error("Wrong UTM zone");
    near(centralMeridian.east, 500000, 1e-6);
    near(centralMeridian.north, 0, 1e-6);

    for (const auto point : {geo::Point{43, 1, 0}, {-33, 18, 0}, {0, -180, 0}, {60, 180, 0}}) {
      const auto utm = geo::toUtm(point);
      const auto restored = geo::fromUtm(utm.east, utm.north, utm.zone);
      near(restored.lat, point.lat, 1e-9);
      near(restored.lon, point.lon, 1e-9);
      if (point.lat < 0 && utm.north >= 0) throw std::runtime_error("Southern northing must be signed");
    }

    const auto ecef = geo::toEcef({53.80939444, 2.12955, 73});
    near(ecef[0], 3771793.968, .001);
    near(ecef[1], 140253.342, .001);
    near(ecef[2], 5124304.349, .001);
    const auto local = geo::fromEnu({3652755.3058, 319574.6799, 5201547.3536},
                                  -189013.869, -128642.040, -4220.171);
    near(local.lat, 53.80939444, 1e-7);
    near(local.lon, 2.12955, 1e-7);
    near(local.alt, 73, .002);

    const auto origin = geo::fromEcef({6378137, 0, 0});
    near(origin.lat, 0, 1e-12);
    near(origin.lon, 0, 1e-12);
    near(origin.alt, 0, 1e-6);
    const auto moved = geo::toEcef(geo::fromEnu({6378137, 0, 0}, 100, 200, 30));
    near(moved[0], 6378137 + 30, 1e-6);
    near(moved[1], 100, 1e-6);
    near(moved[2], 200, 1e-6);
    const auto pole = geo::fromEcef({0, 0, 6356752.314245});
    near(pole.lat, 90, 1e-10);
    near(pole.alt, 0, 1e-6);
    const auto polarOffset = geo::offset({90, 45, 100}, 100, 50, 10);
    near(polarOffset.alt, 110, .002);
    near(geo::distance({90, 45, 100}, polarOffset), std::hypot(100, 50), .003);

    const auto east = geo::advance({0, 0, 123}, geo::pi / 2, 1000);
    near(east.lat, 0, 1e-12);
    near(east.lon, 0.008983152841195, 1e-12);
    near(east.alt, 123, 1e-12);
    near(geo::distance({0, 0, 0}, {0, 90, 0}), 10018754.1713946, 1e-6);
    near(geo::distance({0, 179.9998, 0}, {0, -179.9998, 0}), 44.527796317, .001);
    const auto dateline = geo::advance({0, 179.9998, 0}, geo::pi / 2, 100);
    if (dateline.lon >= 0) throw std::runtime_error("Dateline crossing did not wrap longitude");
    near(geo::distance({0, 179.9998, 0}, dateline), 100, 1e-6);

    rejects([] { geo::fromEcef({0, 0, 0}); });
    rejects([] { geo::fromUtm(500000, 0, 0); });
    rejects([] { geo::fromUtm(500000, 0, 61); });
    rejects([] { geo::toUtm({91, 0, 0}); });
    rejects([] { geo::fromUtm(std::numeric_limits<double>::quiet_NaN(), 0, 31); });
    // An invalid input must not leave a cached PROJ operation in an error state.
    near(geo::fromUtm(500000, 0, 31).lon, 3, 1e-12);
    std::cout << "PROJ coordinate and geodesic checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Geodesy test failed: " << error.what() << '\n';
    return 1;
  }
}
