// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "server_helpers.h"
#include <algorithm>
#include <cmath>

namespace server {
namespace {
// GPS time of week has no week number: use the nearest week to reception.
double unixOfTow(double ms, double now) {
  constexpr double week = 604800, sunday = 259200;
  double base = std::floor((now - sunday) / week) * week + sunday;
  double t = base + ms / 1000;
  if (t - now > week / 2)
    t -= week;
  if (now - t > week / 2)
    t += week;
  return t;
}
}
void Server::updateTelemetry(Aircraft& aircraft, const Message& message, double now) {
  const auto& definition = message.getDefinition();
  const auto& name = definition.getName();
  if (name != "DOWNLINK_STATUS")
    aircraft.lastTelemetry = now;
  if (name == "GPS" || name == "GPS_LLA") {
    aircraft.gpsFix = message.getFieldAs<int>("mode");
    if (name == "GPS_LLA") {
      aircraft.position = {message.getFieldAs<double>("lat") / 1e7, message.getFieldAs<double>("lon") / 1e7, 0};
      aircraft.positionValid = true;
    } else if (aircraft.gpsFix >= 3) {
      aircraft.position = geo::fromUtm(message.getFieldAs<double>("utm_east") / 100,
        message.getFieldAs<double>("utm_north") / 100, message.getFieldAs<int>("utm_zone"));
      aircraft.positionValid = true;
    }
    if (name == "GPS_LLA" || aircraft.gpsFix >= 3) {
      aircraft.timeOfWeek = message.getFieldAs<double>("itow");
      aircraft.unixTime = unixOfTow(aircraft.timeOfWeek, now);
      aircraft.groundSpeed = message.getFieldAs<double>("speed") / 100;
      aircraft.course = geo::normalize(message.getFieldAs<double>("course") * geo::rad / 10);
    }
  } else if (name == "GPS_INT") {
    aircraft.timeOfWeek = message.getFieldAs<double>("tow");
    aircraft.unixTime = unixOfTow(aircraft.timeOfWeek, now);
    aircraft.positionAccuracy = message.getFieldAs<int>("pacc");
  } else if (name == "GPS_SOL")
    aircraft.positionAccuracy = message.getFieldAs<int>("Pacc");
  else if (name == "ESTIMATOR") {
    aircraft.altitude = message.getFieldAs<double>("z");
    aircraft.climbRate = message.getFieldAs<double>("z_dot");
  } else if (name == "ATTITUDE") {
    aircraft.roll = message.getFieldAs<double>("phi");
    aircraft.pitch = message.getFieldAs<double>("theta");
    if (message.getFieldDefinition("phi").getType().getBaseType() == pprzlink::BaseType::INT16) {
      aircraft.roll *= geo::rad;
      aircraft.pitch *= geo::rad;
      aircraft.headingFromCourse = true;
    } else
      aircraft.heading = geo::normalize(message.getFieldAs<double>("psi"));
  } else if (name == "AIRSPEED" || name == "AIR_DATA")
    aircraft.airspeed = message.getFieldAs<double>("airspeed");
  else if (name == "INS_REF") {
    const std::array origin{message.getFieldAs<double>("ecef_x0") / 100, message.getFieldAs<double>("ecef_y0") / 100, message.getFieldAs<double>("ecef_z0") / 100};
    geo::fromEcef(origin);
    aircraft.geoidOffset = (message.getFieldAs<double>("hmsl0") - message.getFieldAs<double>("alt0")) / 1000;
    aircraft.groundAltitude = message.getFieldAs<double>("hmsl0") / 1000;
    aircraft.navigation.reference = EcefReference{origin};
  } else if (name == "NAVIGATION_REF") {
    aircraft.navigation.reference = geo::Utm{message.getFieldAs<double>("utm_east"), message.getFieldAs<double>("utm_north"), message.getFieldAs<int>("utm_zone")};
    aircraft.groundAltitude = message.getFieldAs<double>("ground_alt");
  } else if (name == "NAVIGATION_REF_LLA") {
    aircraft.navigation.reference = geo::Point{message.getFieldAs<double>("lat") / 1e7, message.getFieldAs<double>("lon") / 1e7, 0};
    aircraft.groundAltitude = message.getFieldAs<double>("ground_alt");
  } else if (name == "ROTORCRAFT_FP" || name == "ROTORCRAFT_FP_MIN") {
    aircraft.rotorcraft = true;
    if (std::holds_alternative<EcefReference>(aircraft.navigation.reference)) {
      aircraft.position = relativePosition(aircraft, message.getFieldAs<double>("east") / 256,
        message.getFieldAs<double>("north") / 256, message.getFieldAs<double>("up") / 256);
      aircraft.altitude = aircraft.position.alt;
      aircraft.positionValid = true;
    }
    if (name == "ROTORCRAFT_FP") {
      if (aircraft.navigation.hasReference()) {
        aircraft.navigation.target = relativePosition(aircraft, message.getFieldAs<double>("carrot_east") / 256,
          message.getFieldAs<double>("carrot_north") / 256, message.getFieldAs<double>("carrot_up") / 256);
        aircraft.navigation.altitude = aircraft.groundAltitude + message.getFieldAs<double>("carrot_up") / 256;
      }
      aircraft.navigation.course = message.getFieldAs<double>("carrot_psi") / 4096;
      double east = message.getFieldAs<double>("veast") / 524288,
             north = message.getFieldAs<double>("vnorth") / 524288;
      aircraft.groundSpeed = std::hypot(east, north);
      aircraft.course = geo::normalize(std::atan2(east, north));
      aircraft.climbRate = message.getFieldAs<double>("vup") / 524288;
      aircraft.roll = message.getFieldAs<double>("phi") / 4096;
      aircraft.pitch = message.getFieldAs<double>("theta") / 4096;
      aircraft.heading = geo::normalize(message.getFieldAs<double>("psi") / 4096);
      aircraft.engine.throttle = message.getFieldAs<double>("thrust") / 96;
      aircraft.flightTime = message.getFieldAs<int>("flight_time");
    } else
      aircraft.groundSpeed = message.getFieldAs<double>("gspeed") / 100;
  } else if (name == "DESIRED") {
    if (aircraft.navigation.hasReference())
      aircraft.navigation.target = relativePosition(aircraft,
          message.getFieldAs<double>(definition.hasFieldName("x") ? "x" : "desired_x"),
          message.getFieldAs<double>(definition.hasFieldName("y") ? "y" : "desired_y"));
    aircraft.navigation.altitude = message.getFieldAs<double>(definition.hasFieldName("altitude") ? "altitude" :
      "desired_altitude");
    aircraft.navigation.climbRate = message.getFieldAs<double>(definition.hasFieldName("climb") ? "climb" :
      "desired_climb");
    if (definition.hasFieldName("course"))
      aircraft.navigation.course = message.getFieldAs<double>("course");
  } else if (name == "NAVIGATION" || name == "ROTORCRAFT_NAV_STATUS") {
    aircraft.navigation.block = message.getFieldAs<int>("cur_block");
    aircraft.navigation.stage = message.getFieldAs<int>("cur_stage");
    aircraft.navigation.blockTime = message.getFieldAs<int>("block_time");
    aircraft.navigation.stageTime = message.getFieldAs<int>("stage_time");
    if (definition.hasFieldName("dist2_wp"))
      aircraft.navigation.distanceToWaypoint = std::sqrt(std::max(0., message.getFieldAs<double>("dist2_wp")));
    else if (definition.hasFieldName("dist_wp"))
      aircraft.navigation.distanceToWaypoint = message.getFieldAs<double>("dist_wp");
    if (name == "NAVIGATION") {
      aircraft.flightTime = message.getFieldAs<int>("flight_time");
      aircraft.motorsKilled = message.getFieldAs<int>("kill_auto_throttle") != 0;
    } else
      aircraft.navigation.horizontalMode = message.getFieldAs<int>("horizontal_mode");
  } else if (name == "ENERGY") {
    aircraft.engine.throttle = message.getFieldAs<double>("throttle");
    aircraft.engine.voltage = message.getFieldAs<double>("voltage");
    aircraft.engine.current = message.getFieldAs<double>("current");
    aircraft.engine.charge = message.getFieldAs<double>("charge");
  } else if (name == "ROTORCRAFT_STATUS" || name == "ROVER_STATUS") {
    aircraft.rotorcraft = true;
    aircraft.radio.status = modeName(message.getFieldAs<int>("rc_status"), {"OK", "LOST", "NONE"});
    aircraft.radio.frameRate = message.getFieldAs<int>("frame_rate");
    aircraft.gpsFix = message.getFieldAs<int>("gps_status");
    aircraft.autopilotMode = message.getFieldAs<int>("ap_mode");
    aircraft.motorsKilled = message.getFieldAs<int>("ap_motors_on") == 0;
    aircraft.engine.voltage = message.getFieldAs<double>("vsupply");
  } else if (name == "PPRZ_MODE") {
    aircraft.rotorcraft = false;
    aircraft.autopilotMode = message.getFieldAs<int>("ap_mode");
    aircraft.throttleMode = message.getFieldAs<int>("ap_gaz");
    aircraft.navigation.lateralMode = message.getFieldAs<int>("ap_lateral");
    aircraft.navigation.horizontalMode = message.getFieldAs<int>("ap_horizontal");
    aircraft.calibration.mode = message.getFieldAs<int>("if_calib_mode");
    if (++aircraft.radio.statusAge >= 10) {
      aircraft.radio.statusAge = 10;
      int status = message.getFieldAs<int>("mcu1_status");
      aircraft.radio.status = (status & 1) ? "OK" : (status & 2) ? "NONE" : "LOST";
      aircraft.radio.mode = (status & 8) ? "FAILSAFE" : (status & 4) ? "AUTO" : "MANUAL";
    }
    if (aircraft.radio.mode == "FAILSAFE")
      aircraft.autopilotMode = -2;
  } else if (name == "FBW_STATUS") {
    aircraft.radio.statusAge = 0;
    aircraft.radio.status = modeName(message.getFieldAs<int>("rc_status"), {"OK", "LOST", "NONE"});
    aircraft.radio.mode = modeName(message.getFieldAs<int>("mode"), {"MANUAL", "AUTO", "FAILSAFE"});
    aircraft.radio.frameRate = message.getFieldAs<int>("frame_rate");
  } else if (name == "STATE_FILTER_STATUS")
    aircraft.filterStatus = message.getFieldAs<int>("state_filter_mode");
  else if (name == "DATALINK_REPORT") {
    aircraft.datalink = {
      .uplinkLostTime = message.getFieldAs<std::uint32_t>("uplink_lost_time"),
      .uplinkMessages = message.getFieldAs<std::uint32_t>("uplink_nb_msgs"),
      .downlinkMessages = message.getFieldAs<std::uint32_t>("downlink_nb_msgs"),
      .downlinkRate = message.getFieldAs<std::uint32_t>("downlink_rate")
    };
  } else if (name == "CALIBRATION")
    aircraft.engine.throttleAccumulator = message.getFieldAs<double>("climb_sum_err");
  else if (name == "SETTINGS") {
    aircraft.calibration.value1 = message.getFieldAs<double>("slider_1_val");
    aircraft.calibration.value2 = message.getFieldAs<double>("slider_2_val");
  } else if (name == "CAM") {
    aircraft.camera.pan = message.getFieldAs<double>("pan") * geo::rad;
    aircraft.camera.tilt = message.getFieldAs<double>("tilt") * geo::rad;
    aircraft.camera.targetEast = message.getFieldAs<double>("target_x");
    aircraft.camera.targetNorth = message.getFieldAs<double>("target_y");
  } else if (name == "DL_VALUE") {
    int idx = message.getFieldAs<int>("index");
    if (idx < 0 || idx >= 256)
      throw std::runtime_error("Setting index out of range");
    aircraft.settings[idx] = message.getFieldAs<double>("value");
  } else if (name == "SVINFO") {
    int idx = message.getFieldAs<int>("chn");
    if (idx < 0 || idx >= 40)
      throw std::runtime_error("Satellite channel out of range");
    aircraft.satellites[idx] = {
      .id = message.getFieldAs<int>("SVID"),
      .flags = message.getFieldAs<int>("Flags"),
      .quality = message.getFieldAs<int>("QI"),
      .signalStrength = message.getFieldAs<int>("CNO"),
      .elevation = message.getFieldAs<int>("Elev"),
      .azimuth = message.getFieldAs<int>("Azim"),
      .updatedAt = now
    };
  } else if (name == "WP_MOVED_LLA")
    aircraft.waypoints[message.getFieldAs<int>("wp_id")] = {message.getFieldAs<double>("lat") / 1e7, message.getFieldAs<double>("lon") / 1e7, message.getFieldAs<double>("alt") / 1000};
  else if (name == "WP_MOVED_ENU" && std::holds_alternative<EcefReference>(aircraft.navigation.reference))
    aircraft.waypoints[message.getFieldAs<int>("wp_id")] = relativePosition(aircraft,
      message.getFieldAs<double>("east") / 256, message.getFieldAs<double>("north") / 256,
      message.getFieldAs<double>("up") / 256);
  else if (name == "WP_MOVED" && std::holds_alternative<geo::Utm>(aircraft.navigation.reference)) {
    auto p = geo::fromUtm(message.getFieldAs<double>("utm_east"), message.getFieldAs<double>("utm_north"),
      definition.hasFieldName("utm_zone") ? message.getFieldAs<int>("utm_zone") : std::get<geo::Utm>
      (aircraft.navigation.reference).zone);
    p.alt = message.getFieldAs<double>("alt");
    aircraft.waypoints[message.getFieldAs<int>("wp_id")] = p;
  } else if (name == "CIRCLE") {
    aircraft.navigation.circle = Circle{message.getFieldAs<double>("center_east"), message.getFieldAs<double>("center_north"), message.getFieldAs<double>("radius")};
    aircraft.navigation.segment.reset();
  } else if (name == "SEGMENT") {
    aircraft.navigation.segment = Segment{message.getFieldAs<double>("segment_east_1"), message.getFieldAs<double>("segment_north_1"), message.getFieldAs<double>("segment_east_2"), message.getFieldAs<double>("segment_north_2")};
    aircraft.navigation.circle.reset();
  } else if (name == "SURVEY") {
    aircraft.navigation.survey = Survey{message.getFieldAs<double>("west"), message.getFieldAs<double>("south"), message.getFieldAs<double>("east"), message.getFieldAs<double>("north"), now};
  } else if (name == "MINIMAL_COM" || name == "GENERIC_COM") {
    if (name == "GENERIC_COM" && message.getFieldAs<int>("flight_time") < aircraft.flightTime)
      return;
    bool minimal = name == "MINIMAL_COM";
    aircraft.position = {message.getFieldAs<double>("lat") / (minimal ? 1 : 1e7), message.getFieldAs<double>("lon") / (minimal ? 1 : 1e7), 0};
    aircraft.positionValid = true;
    aircraft.altitude = message.getFieldAs<double>(minimal ? "hmsl" : "alt");
    aircraft.groundSpeed = message.getFieldAs<double>("gspeed") / (minimal ? 1 : 100);
    aircraft.course = geo::normalize(message.getFieldAs<double>("course") / (minimal ? 1 : 1000));
    if (!aircraft.rotorcraft)
      aircraft.heading = aircraft.course;
    if (minimal) {
      aircraft.climbRate = message.getFieldAs<double>("climb");
      aircraft.gpsFix = message.getFieldAs<int>("gps_mode");
    }
    aircraft.engine.voltage = message.getFieldAs<double>("vsupply") / (minimal ? 1 : 10);
    aircraft.engine.throttle = message.getFieldAs<double>("throttle");
    aircraft.autopilotMode = message.getFieldAs<int>("ap_mode");
    aircraft.navigation.block = message.getFieldAs<int>("nav_block");
    aircraft.flightTime = message.getFieldAs<int>("flight_time");
  } else if (name == "FORMATION_SLOT_TM" || name == "FORMATION_STATUS_TM") {
    auto out = makeMessage(name.substr(0, name.size() - 3), "ground_dl");
    for (size_t j = 0; j < out.getDefinition().getNbFields(); ++j) {
      auto key = out.getFieldDefinition(j).getName();
      copyField(out, message, key, key);
    }
    send_(out);
  } else if (name == "TCAS_RA")
    send_(makeMessage("TCAS_RESOLVE", "ground_dl").setField(
      "ac_id", message.getFieldAs<double>("ac_id"),
      "ac_id_conflict", std::stoi(aircraft.id),
      "resolve", message.getFieldAs<double>("resolve")
    ));
  else if (name == "DRAW") {
    static const char *colors[] = {"black", "red", "green", "yellow", "blue", "magenta", "cyan", "white"};
    int color = message.getFieldAs<int>("color");
    auto out = makeMessage("SHAPE").setField(
      "linecolor", std::string(colors[(color >> 3) & 7]),
      "fillcolor", std::string(colors[color & 7]),
      "opacity", (color >> 6) & 3
    );
    for (size_t j = 0; j < message.getDefinition().getNbFields(); ++j) {
      auto key = message.getFieldDefinition(j).getName();
      if (key != "color")
        copyField(out, message, key, key);
    }
    send_(out);
  }
  if (aircraft.headingFromCourse && (name == "GPS" || name == "GPS_LLA" || name == "GENERIC_COM"))
    aircraft.heading = aircraft.course;
  if ((name == "GPS" || name == "GPS_LLA" || name == "MINIMAL_COM") && aircraft.groundSpeed > 3
    && aircraft.autopilotMode == 2) {
    int bin = static_cast<int>(geo::normalize(aircraft.course) * 36 / (2 * geo::pi)) % 36;
    aircraft.wind[bin] = std::array<double, 2> {aircraft.groundSpeed *std::sin(aircraft.course), aircraft.groundSpeed*std::cos(aircraft.course)};
  }
}
}
