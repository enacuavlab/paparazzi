// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace server {
bool Server::registerAircraft(const std::string& id, const Message& alive, double now) {
  if (unknownAircraft_.contains(id)) {
    trace("IGNORE aircraft " + id + ": ID was not found in conf.xml");
    return false;
  }
  trace("REGISTER aircraft " + id);
  try {
    const auto config = configuration_.aircraft(id);
    Aircraft aircraft;
    aircraft.id = id;
    aircraft.name = config.name;
    aircraft.lastTelemetry = now;
    trace("CONFIG aircraft " + id + " name=" + config.name + " directory=" + config.directory.string());
    if (!config.replay && !options_.noMd5) {
      trace("MD5 checking " + config.signature.string());
      const auto expected = configuration_.signature(config);
      const auto digest = alive.getField<std::vector<uint8_t>>("md5sum");
      if (digest.size() != 16)
        throw std::runtime_error("Invalid ALIVE signature");
      const bool zero = std::all_of(digest.begin(), digest.end(), [](auto b) { return b == 0; });
      std::ostringstream actual;
      actual << std::hex << std::setfill('0');
      for (auto b : digest)
        actual << std::setw(2) << int(b);
      trace("MD5 aircraft " + id + " expected=" + expected + " received=" + actual.str());
      if (!zero && expected != actual.str())
        throw std::runtime_error("Configuration MD5 mismatch for " + id);
      if (zero)
        std::cerr << "Warning: zero configuration MD5 for " << id << '\n';
    } else {
      trace("MD5 check skipped for " + id + (config.replay ? " (replay)" : " (--no-md5-check)"));
    }
    trace("LOAD flight plan " + config.flightPlan.string());
    trace("LOAD airframe " + config.builtAirframe.string());
    const auto parameters = configuration_.parameters(config);
    aircraft.groundAltitude = parameters.groundAltitude;
    aircraft.altitude = parameters.groundAltitude;
    aircraft.position = {parameters.latitude, parameters.longitude, 0};
    if (parameters.catastrophicBatteryVoltage)
      aircraft.batteryLimits.catastrophic = *parameters.catastrophicBatteryVoltage;
    if (parameters.criticalBatteryVoltage)
      aircraft.batteryLimits.critical = *parameters.criticalBatteryVoltage;
    if (parameters.lowBatteryVoltage)
      aircraft.batteryLimits.warning = *parameters.lowBatteryVoltage;
    if (parameters.cameraHorizontalFov)
      aircraft.camera.horizontalFov = *parameters.cameraHorizontalFov * geo::rad;
    if (parameters.cameraVerticalFov)
      aircraft.camera.verticalFov = *parameters.cameraVerticalFov * geo::rad;
    aircraft.autopilotModes = parameters.autopilotModes;
    aircraft_.emplace(id, std::move(aircraft));
    trace("REGISTERED aircraft " + id + " (" + aircraft_.at(id).name + ")");
    if (onAircraftRegistered)
      onAircraftRegistered(id);
    send_(makeMessage("NEW_AIRCRAFT").setField("ac_id", id));
    return true;
  } catch (const std::exception& e) {
    // Only unknown IDs are suppressed; builds/signature problems can be fixed while running.
    if (!id.starts_with("replay")) {
      try {
        configuration_.aircraft(id);
      } catch (...) {
        unknownAircraft_.insert(id);
      }
    }
    std::cerr << "Register " << id << ": " << e.what() << '\n';
    return false;
  }
}
Message Server::answer(const Message& request) const {
  if (request.getDefinition().getName() == "AIRCRAFTS_REQ") {
    std::string ids;
    for (const auto& [id, a] : aircraft_)
      ids += id + ',';
    // An empty list is represented by a comma, as in server.ml.
    if (ids.empty())
      ids = ",";
    return makeMessage("AIRCRAFTS").setField("ac_list", ids);
  }
  const auto id = request.getField<std::string>("ac_id");
  const auto config = configuration_.aircraft(id);
  const std::string base = options_.http
    ? "http://" + options_.hostname + ":" + std::to_string(options_.port) + (config.replay ? "/var/replay" : "")
    : "file://" + config.root.string();
  const auto url = [&](const std::filesystem::path& path) {
    return base + "/" + path.lexically_relative(config.root).generic_string();
  };
  return makeMessage("CONFIG").setField(
    "ac_id", id,
    "flight_plan", url(config.flightPlan),
    "airframe", url(config.airframe),
    "radio", url(config.radio),
    "settings", url(config.settings),
    "default_gui_color", config.guiColor,
    "ac_name", config.name
  );
}
}
