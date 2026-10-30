// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "server_helpers.h"
#include "telemetry_json.h"
#include <algorithm>
#include <iostream>
#include <utility>

namespace server {
Server::Server(Options options, const paparazzi::Configuration& configuration,
  const pprzlink::MessageDictionary& dictionary,
  std::function<void(const Message&)> send, std::ostream* dataLog, double start)
  : options_(std::move(options)), dictionary_(dictionary), send_(std::move(send)),
    dataLog_(dataLog), startedAt_(start), configuration_(configuration) {}
Message Server::makeMessage(const std::string& name, const std::string& sender) const {
  Message result(dictionary_.getDefinition(name));
  result.setSenderId(sender);
  return result;
}
void Server::trace(const std::string& message) const {
  if (options_.verbose)
    std::cerr << "[server_cpp] " << message << '\n';
}
void Server::receive(const std::string& sender, const Message& m, double now) {
  const auto& name = m.getDefinition().getName();
  if (m.getDefinition().getClassId() == dictionary_.getClassId("telemetry")) {
    if (!aircraft_.contains(sender)) {
      if (name != "ALIVE" && !(options_.replayOld && name == "PPRZ_MODE")) {
        trace("IGNORE " + sender + " " + name + ": waiting for ALIVE");
        return;
      }
      if (!registerAircraft(sender, m, now))
        return;
    }
    try {
      validateFiniteFields(m);
      logMessage(sender, m, now - startedAt_);
      if (streamJson)
        streamJson(serializeTelemetryJson(m, aircraft_.at(sender).name + " (" + sender + ")", now - startedAt_));
      updateTelemetry(aircraft_.at(sender), m, now);
    } catch (const std::exception& e) {
      std::cerr << sender << ' ' << name << ": " << e.what() << '\n';
      std::string error = e.what();
      std::replace(error.begin(), error.end(), ' ', '_');
      send_(makeMessage("TELEMETRY_ERROR").setField(
        "ac_id", sender,
        "message", error
      ));
    }
  } else if (m.getDefinition().getClassId() == dictionary_.getClassId("ground"))
    handleCommand(m, now);
}
geo::Point Server::relativePosition(const Aircraft& aircraft, double east, double north, double up) const {
  const auto& reference = aircraft.navigation.reference;
  if (const auto* local = std::get_if<EcefReference>(&reference)) {
    auto point = geo::fromEnu(local->origin, east, north, up);
    point.alt += aircraft.geoidOffset;
    return point;
  }
  if (const auto* utm = std::get_if<geo::Utm>(&reference)) {
    auto point = geo::fromUtm(utm->east + east, utm->north + north, utm->zone);
    point.alt = aircraft.groundAltitude + up;
    return point;
  }
  auto origin = std::get<geo::Point>(reference);
  origin.alt = aircraft.groundAltitude - aircraft.geoidOffset;
  auto point = geo::offset(origin, east, north, up);
  point.alt = aircraft.groundAltitude + up;
  return point;
}
}
