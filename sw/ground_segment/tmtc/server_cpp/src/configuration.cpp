// SPDX-License-Identifier: GPL-2.0-or-later
#include "configuration.h"
#include "xml_helpers.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <set>
#include <stdexcept>
#include <utility>

namespace fs = std::filesystem;
namespace paparazzi {
using xml::attr;
using xml::load;
using xml::visitXml;

Configuration::Configuration(fs::path home) : home_(std::move(home)) {
  load(live_, home_ / "conf/conf.xml");
  if (!live_.RootElement())
    throw std::runtime_error("Missing root element in conf.xml");
}

AircraftConfiguration Configuration::aircraft(const std::string& id) const {
  AircraftConfiguration result;
  result.id = id;
  result.replay = id.starts_with("replay");
  result.root = result.replay ? home_ / "var/replay" : home_;
  const auto realId = result.replay ? id.substr(6) : id;
  tinyxml2::XMLDocument replay;
  const auto* document = &live_;
  if (result.replay) {
    load(replay, result.root / "conf/conf.xml");
    document = &replay;
  }
  const auto* root = document->RootElement();
  for (auto* element = root ? root->FirstChildElement("aircraft") : nullptr;
       element; element = element->NextSiblingElement("aircraft")) {
    if (attr(element, "ac_id") != realId)
      continue;
    result.name = attr(element, "name");
    result.guiColor = attr(element, "gui_color", "#ffffff");
    result.directory = result.root / "var/aircrafts" / result.name;
    result.flightPlan = result.directory / "flight_plan.xml";
    result.settings = result.directory / "settings.xml";
    result.signature = result.directory / "conf/aircraft.md5";
    result.airframe = result.root / "conf" / attr(element, "airframe");
    result.radio = result.root / "conf" / attr(element, "radio");
    result.builtAirframe = result.directory / "conf" / attr(element, "airframe");
    if (!fs::exists(result.builtAirframe))
      result.builtAirframe = result.airframe;
    return result;
  }
  throw std::runtime_error("Aircraft '" + id + "' not found in " +
                           (result.root / "conf/conf.xml").string());
}

std::string Configuration::signature(const AircraftConfiguration& aircraft) const {
  std::ifstream input(aircraft.signature);
  std::string expected;
  input >> expected;
  if (!input || expected.size() != 32)
    throw std::runtime_error("Missing/invalid aircraft.md5 for " + aircraft.id);
  std::transform(expected.begin(), expected.end(), expected.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return expected;
}

AircraftParameters Configuration::parameters(const AircraftConfiguration& aircraft) const {
  AircraftParameters result;
  tinyxml2::XMLDocument fp, af;
  load(fp, aircraft.flightPlan);
  const auto* plan = fp.RootElement();
  if (plan && std::string(plan->Name()) != "flight_plan")
    plan = plan->FirstChildElement("flight_plan");
  if (!plan)
    throw std::runtime_error("Missing flight_plan element for " + aircraft.id);
  result.groundAltitude = plan->DoubleAttribute("ground_alt");
  result.latitude = plan->DoubleAttribute("lat0");
  result.longitude = plan->DoubleAttribute("lon0");
  load(af, aircraft.builtAirframe);
  std::set<std::string> autopilots;
  visitXml(af.RootElement(), [&](const tinyxml2::XMLElement * section) {
    if (std::string(section->Name()) == "autopilot")
      autopilots.insert(attr(section, "name"));
    if (std::string(section->Name()) != "section")
      return;
    for (auto * def = section->FirstChildElement("define"); def; def = def->NextSiblingElement("define")) {
      double value;
      if (def->QueryDoubleAttribute("value", &value) != tinyxml2::XML_SUCCESS)
        continue;
      auto n = attr(def, "name");
      if (attr(section, "name") == "BAT") {
        if (n == "CATASTROPHIC_BAT_LEVEL")
          result.catastrophicBatteryVoltage = value;
        if (n == "CRITIC_BAT_LEVEL")
          result.criticalBatteryVoltage = value;
        if (n == "LOW_BAT_LEVEL")
          result.lowBatteryVoltage = value;
      }
      if (attr(section, "name") == "CAM") {
        if (n == "CAM_HFV")
          result.cameraHorizontalFov = value;
        if (n == "CAM_VFV")
          result.cameraVerticalFov = value;
      }
    }
  });
  if (autopilots.size() == 1) {
    std::string filename = *autopilots.begin();
    if (!filename.ends_with(".xml"))
      filename += ".xml";
    tinyxml2::XMLDocument ap;
    if (ap.LoadFile((aircraft.root / "conf/autopilot" / filename).c_str()) == tinyxml2::XML_SUCCESS)
      visitXml(ap.RootElement(), [&](const tinyxml2::XMLElement * machine) {
      auto gcs = attr(machine, "gcs_mode");
      std::transform(gcs.begin(), gcs.end(), gcs.begin(), [](unsigned char c) {
        return std::toupper(c);
      });
      if (std::string(machine->Name()) == "state_machine" && gcs == "TRUE")
        for (auto * mode = machine->FirstChildElement("mode"); mode; mode = mode->NextSiblingElement("mode"))
          result.autopilotModes.push_back(attr(mode, "shortname", attr(mode, "name").c_str()));
    });
  }
  return result;
}

void Configuration::writeSnapshot(const fs::path& path, const fs::path& messageDefinitions,
                                  const std::string& dataFile, double start) const {
  tinyxml2::XMLDocument out;
  auto* config = out.NewElement("configuration");
  out.InsertEndChild(config);
  config->SetAttribute("time_of_day", start);
  config->SetAttribute("data_file", dataFile.c_str());
  auto* c = live_.RootElement()->DeepClone(&out)->ToElement();
  config->InsertEndChild(c);
  // Store source configuration plus generated flight plan/settings for replay.
  for (auto * ac = c->FirstChildElement("aircraft"); ac; ac = ac->NextSiblingElement("aircraft")) {
    const auto name = attr(ac, "name");
    const std::array<fs::path, 4> files = {home_ / "conf" / attr(ac, "airframe"), home_ / "conf" / attr(ac, "radio"),
      home_ / "var/aircrafts" / name / "flight_plan.xml", home_ / "var/aircrafts" / name / "settings.xml"
    };
    for (const auto& file : files) {
      tinyxml2::XMLDocument source;
      if (source.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS || !source.RootElement())
        continue;
      auto* node = source.RootElement()->DeepClone(&out)->ToElement();
      // play_core.ml consumes dumped plans directly and looks for this exact
      // settings tag when reconstructing var/replay.
      if (file.filename() == "settings.xml")
        node->SetName("generated_settings");
      ac->InsertEndChild(node);
    }
  }
  tinyxml2::XMLDocument messages;
  load(messages, messageDefinitions);
  auto* protocol = messages.RootElement();
  if (protocol && std::string(protocol->Name()) != "protocol")
    protocol = protocol->FirstChildElement("protocol");
  if (!protocol)
    throw std::runtime_error("Missing protocol XML");
  config->InsertEndChild(protocol->DeepClone(&out));
  if (out.SaveFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Cannot write " + path.string());
}
}
