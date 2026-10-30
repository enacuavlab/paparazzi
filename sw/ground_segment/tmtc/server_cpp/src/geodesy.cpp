// SPDX-License-Identifier: GPL-2.0-or-later
#include "geodesy.h"

#include <geodesic.h>
#include <proj.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace geo {
namespace {
using Context = std::unique_ptr<PJ_CONTEXT, decltype(&proj_context_destroy)>;
using Projection = std::unique_ptr<PJ, decltype(&proj_destroy)>;

void validate(Point point) {
  if (!std::isfinite(point.lat) || !std::isfinite(point.lon) || !std::isfinite(point.alt) ||
      point.lat < -90 || point.lat > 90 || point.lon < -180 || point.lon > 180)
    throw std::invalid_argument("Invalid geographic coordinate");
}

void validateEcef(std::array<double, 3> point) {
  if (!std::ranges::all_of(point, [](double value) { return std::isfinite(value); }) ||
      std::hypot(point[0], point[1], point[2]) < 1)
    throw std::invalid_argument("Invalid ECEF origin");
}

// Each thread owns its PROJ context and reuses its projections. Local-frame
// origins change in flight, so retain only the most recent ENU transform.
class Transformations {
public:
  Transformations() : context_(proj_context_create(), proj_context_destroy) {
    if (!context_) throw std::runtime_error("Cannot create PROJ context");
    proj_log_level(context_.get(), PJ_LOG_NONE);
    cartesian_ = create("+proj=cart +ellps=WGS84");
    geod_init(&geodesic, 6378137.0, 1.0 / 298.257223563);
  }

  PJ_COORD cartesian(PJ_DIRECTION direction, PJ_COORD coordinate) {
    return apply(cartesian_.get(), direction, coordinate);
  }

  PJ_COORD utm(int zone, PJ_DIRECTION direction, PJ_COORD coordinate) {
    if (zone < 1 || zone > 60) throw std::invalid_argument("Invalid UTM zone");
    auto found = utm_.find(zone);
    if (found == utm_.end()) {
      // Deliberately omit +south: Paparazzi uses signed northings, without the
      // southern hemisphere's conventional 10,000,000 m false northing.
      auto operation = create("+proj=utm +ellps=WGS84 +zone=" + std::to_string(zone));
      found = utm_.emplace(zone, std::move(operation)).first;
    }
    return apply(found->second.get(), direction, coordinate);
  }

  PJ_COORD enu(std::array<double, 3> origin, PJ_COORD coordinate) {
    if (!enuOrigin_ || *enuOrigin_ != origin) {
      std::ostringstream definition;
      definition.imbue(std::locale::classic());
      definition << std::setprecision(17)
                 << "+proj=pipeline +step +proj=cart +ellps=WGS84"
                 << " +step +proj=topocentric +ellps=WGS84"
                 << " +X_0=" << origin[0] << " +Y_0=" << origin[1] << " +Z_0=" << origin[2];
      enu_ = create(definition.str());
      enuOrigin_ = origin;
    }
    return apply(enu_.get(), PJ_INV, coordinate);
  }

  geod_geodesic geodesic{};

private:
  Context context_;
  Projection cartesian_{nullptr, proj_destroy};
  std::map<int, Projection> utm_;
  Projection enu_{nullptr, proj_destroy};
  std::optional<std::array<double, 3>> enuOrigin_;

  std::string error(int code) const {
    const char* description = proj_context_errno_string(context_.get(), code);
    return description ? description : "Unknown PROJ error";
  }

  Projection create(const std::string& definition) {
    Projection operation(proj_create(context_.get(), definition.c_str()), proj_destroy);
    if (!operation)
      throw std::runtime_error("Cannot create PROJ transformation: " + error(proj_context_errno(context_.get())));
    return operation;
  }

  PJ_COORD apply(PJ* operation, PJ_DIRECTION direction, PJ_COORD coordinate) {
    proj_errno_reset(operation);
    auto result = proj_trans(operation, direction, coordinate);
    if (const int code = proj_errno(operation))
      throw std::runtime_error("PROJ transformation failed: " + error(code));
    if (!std::isfinite(result.xyz.x) || !std::isfinite(result.xyz.y) || !std::isfinite(result.xyz.z))
      throw std::runtime_error("PROJ transformation returned an invalid coordinate");
    return result;
  }
};

Transformations& transformations() {
  static thread_local Transformations instance;
  return instance;
}

Point geographic(PJ_COORD coordinate) {
  return {coordinate.lpz.phi / rad, coordinate.lpz.lam / rad, coordinate.lpz.z};
}
} // namespace

double normalize(double radians) {
  const double angle = std::fmod(radians, 2 * pi);
  return angle < 0 ? angle + 2 * pi : angle;
}

Point fromEcef(std::array<double, 3> ecef) {
  validateEcef(ecef);
  return geographic(transformations().cartesian(PJ_INV, proj_coord(ecef[0], ecef[1], ecef[2], 0)));
}

std::array<double, 3> toEcef(Point point) {
  validate(point);
  const auto coordinate = transformations().cartesian(PJ_FWD, proj_coord(point.lon * rad, point.lat * rad, point.alt, 0));
  return {coordinate.xyz.x, coordinate.xyz.y, coordinate.xyz.z};
}

Point fromEnu(std::array<double, 3> origin, double east, double north, double up) {
  validateEcef(origin);
  return geographic(transformations().enu(origin, proj_coord(east, north, up, 0)));
}

Point offset(Point origin, double east, double north, double up) {
  return fromEnu(toEcef(origin), east, north, up);
}

Point fromUtm(double east, double north, int zone) {
  return geographic(transformations().utm(zone, PJ_INV, proj_coord(east, north, 0, 0)));
}

Utm toUtm(Point point) {
  validate(point);
  const int zone = std::clamp(static_cast<int>(std::floor((point.lon + 180) / 6)) + 1, 1, 60);
  const auto coordinate = transformations().utm(zone, PJ_FWD, proj_coord(point.lon * rad, point.lat * rad, 0, 0));
  return {coordinate.xy.x, coordinate.xy.y, zone};
}

Point advance(Point origin, double bearing, double distance) {
  validate(origin);
  if (!std::isfinite(bearing) || !std::isfinite(distance))
    throw std::invalid_argument("Invalid geodesic bearing or distance");
  Point result = origin;
  geod_direct(&transformations().geodesic, origin.lat, origin.lon, bearing / rad, distance,
              &result.lat, &result.lon, nullptr);
  return result;
}

double distance(Point from, Point to) {
  validate(from);
  validate(to);
  double metres = 0;
  geod_inverse(&transformations().geodesic, from.lat, from.lon, to.lat, to.lon, &metres, nullptr, nullptr);
  return metres;
}
} // namespace geo
