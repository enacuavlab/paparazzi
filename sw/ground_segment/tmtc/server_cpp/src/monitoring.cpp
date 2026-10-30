// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "server_helpers.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace server {
void Server::checkAlerts(double now) {
  for (auto it = aircraft_.begin(); it != aircraft_.end(); ++it) {
    const auto& [id, aircraft] = *it;
    std::string level = aircraft.engine.voltage < 1 ? "INVALID" : aircraft.engine.voltage <
      aircraft.batteryLimits.catastrophic ? "CATASTROPHIC" : aircraft.engine.voltage <
      aircraft.batteryLimits.critical ?
    "CRITIC" : aircraft.engine.voltage < aircraft.batteryLimits.warning ? "WARNING" : "";
    if (!level.empty())
      send_(makeMessage("BAT_LOW").setField(
        "ac_id", id,
        "level", level,
        "value", aircraft.engine.voltage
      ));
    if (!aircraft.positionValid)
      continue;
    for (auto jt = std::next(it); jt != aircraft_.end(); ++jt) {
      const auto& other = jt->second;
      if (!other.positionValid || std::abs(aircraft.altitude - other.altitude) >= 10)
        continue;
      const double separation = geo::distance(aircraft.position, other.position);
      if (separation >= 100)
        continue;
      const auto predictedAircraft = geo::advance(aircraft.position, aircraft.course, aircraft.groundSpeed * .2);
      const auto predictedOther = geo::advance(other.position, other.course, other.groundSpeed * .2);
      const bool approaching = geo::distance(predictedAircraft, predictedOther) < separation;
      send_(makeMessage("AIR_PROX").setField(
        "ac_id", id + "," + other.id,
        "level", std::string(approaching ? "CRITICAL" : "WARNING")
      ));
    }
  }
  std::erase_if(intruders_, [now](const auto& entry) { return now - entry.second.updatedAt > 10; });
  for (const auto& [id, intruder] : intruders_) {
    try {
      const int trafficId = 200 + (std::stoi(id) % 50);
      publishTraffic(std::to_string(trafficId), intruder.position, intruder.position.alt,
                     intruder.course, intruder.speed, intruder.climbRate, intruder.timeOfWeek, false);
    } catch (const std::exception& error) {
      std::cerr << "Intruder: " << error.what() << '\n';
    }
  }
}
void Server::publishWind() {
  // Fit a circle to ground-speed vectors; centre is wind and radius is airspeed.
  // Keep one sample per heading bin instead of an unbounded sample history.
  for (const auto& [id, aircraft] : aircraft_) {
    std::vector<std::array<double, 2>> samples;
    for (const auto& sample : aircraft.wind)
      if (sample) samples.push_back(*sample);
    if (samples.size() < 3) continue;

    double x = 0, y = 0;
    for (auto sample : samples) {
      x += sample[0];
      y += sample[1];
    }
    x /= samples.size();
    y /= samples.size();
    double xx = 0, xy = 0, yy = 0, xr = 0, yr = 0;
    for (auto sample : samples) {
      double u = sample[0] - x, v = sample[1] - y, r = u*u + v*v;
      xx += u*u;
      xy += u*v;
      yy += v*v;
      xr += u*r;
      yr += v*r;
    }
    double determinant = xx*yy - xy*xy;
    if (std::abs(determinant) < 1e-8) continue;
    double wx = x + (xr*yy - yr*xy) / (2*determinant);
    double wy = y + (yr*xx - xr*xy) / (2*determinant);
    std::vector<double> airspeeds;
    double mean = 0;
    for (auto sample : samples) {
      double speed = std::hypot(sample[0] - wx, sample[1] - wy);
      airspeeds.push_back(speed);
      mean += speed;
    }
    mean /= airspeeds.size();
    double variance = 0;
    for (double speed : airspeeds)
      variance += (speed - mean) * (speed - mean);
    variance /= airspeeds.size();
    send_(makeMessage("WIND").setField(
      "ac_id", id,
      "dir", geo::normalize(std::atan2(-wx, -wy)) / geo::rad,
      "wspeed", std::hypot(wx, wy),
      "mean_aspeed", mean,
      "stddev", std::sqrt(variance)
    ));
  }
}
}
