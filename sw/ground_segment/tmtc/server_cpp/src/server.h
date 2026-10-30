// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include "aircraft.h"
#include "configuration.h"
#include "options.h"
#include <pprzlink/MessageDictionary.h>
#include <pprzlink/Message.h>
#include <filesystem>
#include <functional>
#include <map>
#include <ostream>
#include <set>

namespace server {
using Message = pprzlink::Message;
// Based on server.ml and parse_messages_v1.ml (CENA/ENAC, Pascal Brisset
// and Antoine Drouin). Camera rotations follow quaternion.ml.
// Core has no threads or sockets. All callbacks and ticks run on the application thread.
class Server {
public:
  Server(Options options, const paparazzi::Configuration& configuration,
         const pprzlink::MessageDictionary& dictionary,
         std::function<void(const Message&)> send, std::ostream* dataLog = nullptr, double start = 0);
  void receive(const std::string& sender, const Message& message, double now);
  Message answer(const Message& request) const;
  void broadcast(double now);
  void checkAlerts(double now);
  void publishWind();
  const std::map<std::string, Aircraft>& aircraft() const { return aircraft_; }
  std::function<void(const std::string&)> streamJson;
  std::function<void(const std::string&)> onAircraftRegistered;

private:
  Message makeMessage(const std::string& name, const std::string& sender = "ground") const;
  Options options_;
  const pprzlink::MessageDictionary& dictionary_;
  std::function<void(const Message&)> send_;
  std::ostream* dataLog_;
  double startedAt_;
  const paparazzi::Configuration& configuration_; // Owned by Application; outlives Server.
  std::map<std::string, Aircraft> aircraft_;
  std::set<std::string> unknownAircraft_;
  std::map<std::string, TrafficReport> intruders_;
  bool registerAircraft(const std::string& id, const Message& alive, double now);
  void updateTelemetry(Aircraft& aircraft, const Message& message, double now);
  void handleCommand(const Message& message, double now);
  void logMessage(const std::string& id, const Message& m, double time);
  void publishAircraft(const Aircraft& aircraft, double now);
  void publishLinkStatus(const Aircraft& aircraft, const std::string& linkId,
                         const LinkStatus* link, double now);
  void publishTraffic(const std::string& id, geo::Point pos, double alt, double course,
               double speed, double climb, double itow, bool lla);
  geo::Point relativePosition(const Aircraft& aircraft, double east, double north, double up = 0) const;
  void trace(const std::string& message) const;
};
}
