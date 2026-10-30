// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "server_helpers.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace server {
namespace {
std::string autopilotModeName(const Aircraft& aircraft) {
  if (aircraft.autopilotMode == -2)
    return "FAIL";
  if (!aircraft.autopilotModes.empty())
    return aircraft.autopilotMode >= 0
      && static_cast<std::size_t>(aircraft.autopilotMode) < aircraft.autopilotModes.size() ?
      aircraft.autopilotModes[aircraft.autopilotMode] : "UNK";
  return aircraft.rotorcraft ? modeName(aircraft.autopilotMode, {"KILL", "SAFE", "HOME", "RATE", "ATT", "R_RCC", "A_RCC", "ATT_C", "R_ZH", "A_ZH", "HOVER", "HOV_C", "H_ZH", "NAV", "RC_D", "CF", "FWD", "MODULE", "FLIP", "GUIDED"})
    : modeName(aircraft.autopilotMode, {"MANUAL", "AUTO1", "AUTO2", "HOME", "NOGPS"});
}
struct Quaternion {
  double w, x, y, z;
};
Quaternion rotation(double roll, double pitch, double yaw) {
  const double cr = std::cos(-roll / 2), sr = std::sin(-roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {
    cy * cr * cp + sy * sr * sp,
    cy * sr * cp - sy * cr * sp,
    cy * cr * sp + sy * sr * cp,
    sy * cr * cp - cy * sr * sp
  };
}

Quaternion multiply(Quaternion a, Quaternion b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x,
    -a.x * b.z + a.y * b.w + a.z * b.x + a.w * b.y,
    a.x * b.y - a.y * b.x + a.z * b.w + a.w * b.z
  };
}
std::array<double, 3> angles(Quaternion q) {
  return {std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
      std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1., 1.)), std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
}
}
void Server::publishTraffic(const std::string& id, geo::Point pos, double alt, double course, double speed,
  double climb, double itow, bool lla) {
  if (lla)
    send_(makeMessage("ACINFO_LLA", "ground_dl").setField(
      "ac_id", aircraftId(id),
      "lat", std::trunc(pos.lat * 1e7),
      "lon", std::trunc(pos.lon * 1e7),
      "course", std::trunc(geo::normalize(course) / geo::rad * 10),
      "alt", std::trunc(alt * 100),
      "speed", std::trunc(speed * 100),
      "climb", std::trunc(climb * 100),
      "itow", itow
    ));
  else {
    auto u = geo::toUtm(pos);
    send_(makeMessage("ACINFO", "ground_dl").setField(
      "ac_id", aircraftId(id),
      "utm_east", std::trunc(u.east * 100),
      "utm_north", std::trunc(u.north * 100),
      "utm_zone", u.zone,
      "course", std::trunc(geo::normalize(course) / geo::rad * 10),
      "alt", std::trunc(alt * 100),
      "speed", std::trunc(speed * 100),
      "climb", std::trunc(climb * 100),
      "itow", itow
    ));
  }
}
void Server::publishAircraft(const Aircraft& aircraft, double now) {
  const auto& id = aircraft.id;
  send_(makeMessage("FLIGHT_PARAM").setField(
    "ac_id", id,
    "roll", aircraft.roll / geo::rad,
    "pitch", aircraft.pitch / geo::rad,
    "heading", aircraft.heading / geo::rad,
    "lat", aircraft.position.lat,
    "long", aircraft.position.lon,
    "speed", aircraft.groundSpeed,
    "airspeed", aircraft.airspeed,
    "course", aircraft.course / geo::rad,
    "alt", aircraft.altitude,
    "climb", aircraft.climbRate,
    "agl", aircraft.altitude - aircraft.groundAltitude,
    "unix_time", aircraft.unixTime,
    "itow", aircraft.timeOfWeek
  ));
  send_(makeMessage("ENGINE_STATUS").setField(
    "ac_id", id,
    "throttle", aircraft.engine.throttle,
    "throttle_accu", aircraft.engine.throttleAccumulator,
    "rpm", aircraft.engine.rpm,
    "temp", aircraft.engine.temperature,
    "bat", aircraft.engine.voltage,
    "amp", aircraft.engine.current,
    "charge", aircraft.engine.charge
  ));
  send_(makeMessage("AP_STATUS").setField(
    "ac_id", id,
    "ap_mode", autopilotModeName(aircraft),
    "lat_mode", modeName(aircraft.navigation.lateralMode, {"MANUAL", "ROLL_RATE", "ROLL", "COURSE"}),
    "horiz_mode", modeName(aircraft.navigation.horizontalMode, {"WAYPOINT", "ROUTE", "CIRCLE", "ATTITUDE", "MANUAL"}),
    "gaz_mode", modeName(aircraft.throttleMode, {"MANUAL", "THROTTLE", "CLIMB", "ALT"}),
    "gps_mode", modeName(aircraft.gpsFix, {"NOFIX", "NA", "2D", "3D", "DGPS", "RTK"}),
    "kill_mode", std::string(aircraft.motorsKilled ? "ON" : "OFF"),
    "flight_time", aircraft.flightTime,
    "state_filter_mode", modeName(aircraft.filterStatus, {"UNKNOWN", "INIT", "ALIGN", "OK", "GPS_LOST", "IMU_LOST", "COV_ERR", "IR_CONTRAST", "ERROR"})
  ));
  send_(makeMessage("FLY_BY_WIRE").setField(
    "ac_id", id,
    "rc_mode", aircraft.radio.mode,
    "rc_status", aircraft.radio.status,
    "rc_rate", aircraft.radio.frameRate
  ));
  send_(makeMessage("INFLIGH_CALIB").setField(
    "ac_id", id,
    "if_mode", modeName(aircraft.calibration.mode, {"OFF", "DOWN", "UP"}),
    "if_value1", aircraft.calibration.value1,
    "if_value2", aircraft.calibration.value2
  ));
  if (aircraft.gpsFix >= 3 && aircraft.navigation.hasReference()) {
    auto attitude = rotation(aircraft.roll, aircraft.pitch, -aircraft.heading);
    auto gimbal = rotation(0, -(aircraft.camera.tilt - geo::pi / 2), -aircraft.camera.pan);
    std::ostringstream lats, lons;
    for (auto corner : std::array<std::array<double, 2>, 4> {{{-1, 1}, {1, 1}, {1, -1}, {-1, -1}}}) {
      auto q = multiply(attitude, multiply(gimbal, rotation(corner[0] * aircraft.camera.horizontalFov / 2,
        corner[1] * aircraft.camera.verticalFov / 2,
        0)));
      auto [roll, pitch, yaw] = angles(q);
      double dx = (aircraft.altitude - aircraft.groundAltitude) * std::tan(std::clamp(roll, -89 * geo::rad,
        89 * geo::rad));
      double dy = (aircraft.altitude - aircraft.groundAltitude) * std::tan(std::clamp(pitch, -89 * geo::rad,
        89 * geo::rad));
      double east = dx * std::cos(yaw) - dy * std::sin(yaw), north = dx * std::sin(yaw) + dy * std::cos(yaw);
      auto origin = aircraft.position;
      origin.alt = aircraft.groundAltitude - aircraft.geoidOffset;
      const auto cornerPosition = geo::offset(origin, east, north);
      lats << std::fixed << std::setprecision(7) << cornerPosition.lat << ',';
      lons << std::fixed << std::setprecision(7) << cornerPosition.lon << ',';
    }
    auto target = relativePosition(aircraft, aircraft.camera.targetEast, aircraft.camera.targetNorth);
    send_(makeMessage("CAM_STATUS").setField(
      "ac_id", id,
      "lats", lats.str(),
      "longs", lons.str(),
      "cam_target_lat", target.lat,
      "cam_target_long", target.lon
    ));
  }
  if (aircraft.navigation.hasReference()) {
    send_(makeMessage("NAV_STATUS").setField(
      "ac_id", id,
      "cur_block", aircraft.navigation.block,
      "cur_stage", aircraft.navigation.stage,
      "block_time", aircraft.navigation.blockTime,
      "stage_time", aircraft.navigation.stageTime,
      "target_lat", aircraft.navigation.target.lat,
      "target_long", aircraft.navigation.target.lon,
      "target_climb", aircraft.navigation.climbRate,
      "target_alt", aircraft.navigation.altitude,
      "target_course", aircraft.navigation.course / geo::rad,
      "dist_to_wp", aircraft.navigation.distanceToWaypoint
    ));
    if (aircraft.navigation.circle && aircraft.navigation.horizontalMode == 2) {
      auto p = relativePosition(aircraft, aircraft.navigation.circle->east, aircraft.navigation.circle->north);
      send_(makeMessage("CIRCLE_STATUS").setField(
        "ac_id", id,
        "circle_lat", p.lat,
        "circle_long", p.lon,
        "radius", std::trunc(aircraft.navigation.circle->radius)
      ));
    }
    if (aircraft.navigation.segment && aircraft.navigation.horizontalMode == 1) {
      auto p = relativePosition(aircraft, aircraft.navigation.segment->east1, aircraft.navigation.segment->north1);
      auto q = relativePosition(aircraft, aircraft.navigation.segment->east2, aircraft.navigation.segment->north2);
      send_(makeMessage("SEGMENT_STATUS").setField(
        "ac_id", id,
        "segment1_lat", p.lat,
        "segment1_long", p.lon,
        "segment2_lat", q.lat,
        "segment2_long", q.lon
      ));
    }
    if (aircraft.navigation.survey && now - aircraft.navigation.survey->updatedAt <= 5) {
      auto p = relativePosition(aircraft, aircraft.navigation.survey->west, aircraft.navigation.survey->south);
      auto q = relativePosition(aircraft, aircraft.navigation.survey->east, aircraft.navigation.survey->north);
      send_(makeMessage("SURVEY_STATUS").setField(
        "ac_id", id,
        "west_long", p.lon,
        "south_lat", p.lat,
        "east_long", q.lon,
        "north_lat", q.lat
      ));
    }
  }
  if (!aircraft.navigation.survey || now - aircraft.navigation.survey->updatedAt > 5)
    send_(makeMessage("SURVEY_STATUS").setField(
      "ac_id", id,
      "west_long", 0.,
      "south_lat", 0.,
      "east_long", 0.,
      "north_lat", 0.
    ));
  if (!aircraft.settings.empty()) {
    std::ostringstream csv;
    for (int j = 0; j <= aircraft.settings.rbegin()->first; ++j) {
      auto s = aircraft.settings.find(j);
      if (s != aircraft.settings.end() && s->second)
        csv << *s->second;
      else
        csv << '?';
      csv << ',';
    }
    send_(makeMessage("DL_VALUES").setField(
      "ac_id", id,
      "values", csv.str()
    ));
  }
  for (const auto& [wp, p] : aircraft.waypoints)
    send_(makeMessage("WAYPOINT_MOVED").setField(
      "ac_id", id,
      "wp_id", wp,
      "lat", p.lat,
      "long", p.lon,
      "alt", p.alt,
      "ground_alt", aircraft.groundAltitude
    ));
  if (aircraft.links.empty()) {
    publishLinkStatus(aircraft, "no_id", nullptr, now);
  } else {
    for (const auto& [linkId, status] : aircraft.links)
      publishLinkStatus(aircraft, linkId, &status, now);
  }
  std::string satelliteIds, flags, quality, signalStrength, elevation, azimuth, ages;
  for (const auto& satellite : aircraft.satellites) {
    satelliteIds += std::to_string(satellite.id) + ',';
    flags += std::to_string(satellite.flags) + ',';
    quality += std::to_string(satellite.quality) + ',';
    signalStrength += std::to_string(satellite.signalStrength) + ',';
    elevation += std::to_string(satellite.elevation) + ',';
    azimuth += std::to_string(satellite.azimuth) + ',';
    const double updatedAt = satellite.updatedAt ? satellite.updatedAt : startedAt_;
    ages += std::to_string(static_cast<int>(std::max(0., now - updatedAt))) + ',';
  }
  send_(makeMessage("SVSINFO").setField(
    "ac_id", id,
    "pacc", aircraft.positionAccuracy,
    "svid", satelliteIds,
    "flags", flags,
    "qi", quality,
    "cno", signalStrength,
    "elev", elevation,
    "azim", azimuth,
    "msg_age", ages
  ));
  if (aircraft_.size() > 1 && aircraft.positionValid && !id.starts_with("replay"))
    publishTraffic(id, aircraft.position,
      aircraft.rotorcraft ? aircraft.altitude - aircraft.geoidOffset : aircraft.altitude, aircraft.course,
      aircraft.groundSpeed, aircraft.climbRate, aircraft.timeOfWeek, aircraft.rotorcraft);
}
void Server::publishLinkStatus(const Aircraft& aircraft, const std::string& linkId,
  const LinkStatus* link, double now) {
  const LinkStatus defaults;
  const auto& status = link ? *link : defaults;
  const auto& datalink = aircraft.datalink;
  const double linkAge = link ? now - status.updatedAt + status.rxLostTime : 0;
  send_(makeMessage("TELEMETRY_STATUS").setField(
    "ac_id", aircraft.id,
    "link_id", linkId,
    "time_since_last_msg", std::max({0., now - aircraft.lastTelemetry, linkAge}),
    "rx_bytes", status.rxBytes,
    "rx_msgs", status.rxMessages,
    "rx_bytes_rate", status.rxBytesRate,
    "tx_msgs", status.txMessages,
    "uplink_lost_time", datalink.uplinkLostTime,
    "uplink_msgs", datalink.uplinkMessages,
    "downlink_msgs", datalink.downlinkMessages,
    "downlink_rate", datalink.downlinkRate,
    "ping_time", status.pingTime
  ));
}

void Server::broadcast(double now) {
  for (const auto& [id, a] : aircraft_) {
    try {
      publishAircraft(a, now);
    } catch (const std::exception& e) {
      std::cerr << "Broadcast " << id << ": " << e.what() << '\n';
    }
  }
}
}
