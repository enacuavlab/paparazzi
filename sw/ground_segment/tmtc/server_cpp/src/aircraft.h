// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include "geodesy.h"
#include <map>
#include <optional>
#include <variant>
#include <vector>
#include <string>
#include <cstdint>

namespace server {
struct EngineState {
  double throttle = 0;
  double throttleAccumulator = 0;
  double rpm = 0;
  double temperature = 0;
  double voltage = 0;
  double current = 0;
  double charge = 0;
};

struct RadioState {
  std::string mode = "UNK";
  std::string status = "UNK";
  int frameRate = 0;
  int statusAge = 10;
};

struct CameraState {
  double pan = 0;
  double tilt = 0;
  double targetEast = 0;
  double targetNorth = 0;
  double horizontalFov = 65 * geo::rad;
  double verticalFov = 35 * geo::rad;
};

struct CalibrationState {
  int mode = 0;
  double value1 = 0;
  double value2 = 0;
};

struct BatteryLimits {
  double catastrophic = 9;
  double critical = 10;
  double warning = 10.5;
};

struct EcefReference {
  std::array<double, 3> origin;
};
using NavigationReference = std::variant<std::monostate, geo::Utm, EcefReference, geo::Point>;

struct Circle { double east, north, radius; };
struct Segment { double east1, north1, east2, north2; };
struct Survey { double west, south, east, north, updatedAt; };

struct NavigationState {
  NavigationReference reference;
  geo::Point target;
  double altitude = 0;
  double climbRate = 0;
  double course = 0;
  double distanceToWaypoint = 0;
  int block = 0;
  int stage = 0;
  int blockTime = 0;
  int stageTime = 0;
  int lateralMode = -1;
  int horizontalMode = -1;
  std::optional<Circle> circle;
  std::optional<Segment> segment;
  std::optional<Survey> survey;

  bool hasReference() const { return !std::holds_alternative<std::monostate>(reference); }
};

struct LinkStatus {
  double updatedAt = 0;
  std::uint32_t rxLostTime = 0;
  std::uint32_t rxBytes = 0;
  std::uint32_t rxMessages = 0;
  double rxBytesRate = 0;
  std::uint32_t txMessages = 0;
  double pingTime = 9999;
};

struct DatalinkStatus {
  std::uint32_t uplinkLostTime = 9999;
  std::uint32_t uplinkMessages = 0;
  std::uint32_t downlinkMessages = 0;
  std::uint32_t downlinkRate = 0;
};

struct TrafficReport {
  geo::Point position;
  double course = 0;
  double speed = 0;
  double climbRate = 0;
  std::uint32_t timeOfWeek = 0;
  double updatedAt = 0;
};

struct Aircraft {
  std::string id;
  std::string name;
  geo::Point position;
  double altitude = 0;
  double groundAltitude = 0;
  double geoidOffset = 0;
  double roll = 0;
  double pitch = 0;
  double heading = 0;
  double groundSpeed = 0;
  double airspeed = -1;
  double course = 0;
  double climbRate = 0;
  double unixTime = 0;
  double timeOfWeek = 0;
  double lastTelemetry = 0;
  int flightTime = 0;
  int autopilotMode = -1;
  int throttleMode = -1;
  int gpsFix = 0;
  int filterStatus = 0;
  int positionAccuracy = 0;
  bool headingFromCourse = false;
  bool rotorcraft = false;
  bool motorsKilled = false;
  bool positionValid = false;
  std::vector<std::string> autopilotModes;
  EngineState engine;
  RadioState radio;
  CameraState camera;
  CalibrationState calibration;
  NavigationState navigation;
  BatteryLimits batteryLimits;
  DatalinkStatus datalink;
  std::map<int, std::optional<double>> settings;
  std::map<int, geo::Point> waypoints; // alt is HMSL here
  std::map<std::string, LinkStatus> links;
  struct Satellite {
    int id = 0;
    int flags = 0;
    int quality = 0;
    int signalStrength = 0; // dB-Hz
    int elevation = 0; // degrees
    int azimuth = 0; // degrees
    double updatedAt = 0;
  };
  std::array<Satellite, 40> satellites{};
  std::array<std::optional<std::array<double, 2>>, 36> wind{};
};

}
