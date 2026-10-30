// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <tinyxml2.h>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace paparazzi {
// Resolved files for a configured aircraft, including replay aircraft. Paths
// are filesystem paths; URL construction and message handling belong to callers.
struct AircraftConfiguration {
  std::string id, name, guiColor;
  bool replay = false;
  std::filesystem::path root, directory;
  std::filesystem::path flightPlan, airframe, radio, settings, signature;
  std::filesystem::path builtAirframe; // Built copy if present, otherwise source.
};

struct AircraftParameters {
  double latitude = 0, longitude = 0, groundAltitude = 0;
  std::optional<double> catastrophicBatteryVoltage, criticalBatteryVoltage, lowBatteryVoltage;
  std::optional<double> cameraHorizontalFov, cameraVerticalFov; // degrees
  std::vector<std::string> autopilotModes;
};

// Configuration files only: no Server, Options, aircraft runtime state,
// PprzLink messages, networking or output callbacks. The live conf.xml is a
// startup snapshot; replay conf.xml is read on lookup, after replay creates it.
class Configuration {
 public:
  explicit Configuration(std::filesystem::path home);
  AircraftConfiguration aircraft(const std::string& id) const;
  AircraftParameters parameters(const AircraftConfiguration& aircraft) const;
  std::string signature(const AircraftConfiguration& aircraft) const;
  void writeSnapshot(const std::filesystem::path& path, const std::filesystem::path& messages,
                     const std::string& dataFile, double startedAt) const;
 private:
  std::filesystem::path home_;
  tinyxml2::XMLDocument live_;
};
}
