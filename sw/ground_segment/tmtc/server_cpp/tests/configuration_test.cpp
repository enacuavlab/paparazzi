// SPDX-License-Identifier: GPL-2.0-or-later
#include "configuration.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;
namespace {
void check(bool condition, const char* description) {
  if (!condition)
    throw std::runtime_error(description);
}
void write(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << contents;
}
template<class F> void fails(F operation, const char* description) {
  try { operation(); }
  catch (const std::runtime_error&) { return; }
  throw std::runtime_error(description);
}
struct Fixture {
  fs::path home;
  Fixture() {
    char pattern[] = "/tmp/paparazzi-configuration-XXXXXX";
    const auto* directory = mkdtemp(pattern);
    if (!directory)
      throw std::runtime_error("mkdtemp failed");
    home = directory;
  }
  ~Fixture() { fs::remove_all(home); }
};
}

int main() {
  try {
    Fixture fixture;
    const auto home = fixture.home;
    write(home / "conf/conf.xml",
      R"(<conf><aircraft ac_id="6" name="Test" airframe="airframes/test.xml" radio="radio.xml"/></conf>)");
    // Lookup needs only conf.xml, so CONFIG replies can precede a build.
    const paparazzi::Configuration configuration(home);
    auto aircraft = configuration.aircraft("6");
    check(aircraft.name == "Test" && aircraft.guiColor == "#ffffff" && !aircraft.replay, "Aircraft metadata");
    check(aircraft.airframe == home / "conf/airframes/test.xml", "Source airframe path");
    check(aircraft.flightPlan == home / "var/aircrafts/Test/flight_plan.xml", "Generated flight plan path");
    fails([&] { configuration.parameters(aircraft); }, "Unbuilt aircraft parameters accepted");
    fails([&] { configuration.aircraft("99"); }, "Unknown aircraft accepted");

    write(aircraft.flightPlan, R"(<dump><flight_plan lat0="43" lon0="1" ground_alt="100"/></dump>)");
    write(aircraft.airframe, R"(<airframe><section name="BAT">
      <define name="LOW_BAT_LEVEL" value="11.5"/></section></airframe>)");
    auto parameters = configuration.parameters(aircraft);
    check(parameters.latitude == 43 && parameters.longitude == 1 && parameters.groundAltitude == 100,
          "Flight plan reference");
    check(parameters.lowBatteryVoltage == 11.5 && !parameters.criticalBatteryVoltage &&
          !parameters.cameraHorizontalFov, "Missing defines must preserve caller defaults");

    const auto built = aircraft.directory / "conf/airframes/test.xml";
    write(built, R"(<airframe><autopilot name="custom"/>
      <section name="BAT"><define name="LOW_BAT_LEVEL" value="12"/>
      <define name="CRITIC_BAT_LEVEL" value="11"/><define name="CATASTROPHIC_BAT_LEVEL" value="10"/></section>
      <section name="CAM"><define name="CAM_HFV" value="80"/><define name="CAM_VFV" value="40"/></section>
      </airframe>)");
    write(home / "conf/autopilot/custom.xml", R"(<autopilot><state_machine gcs_mode="true">
      <mode name="Manual" shortname="MAN"/><mode name="Navigation"/>
      </state_machine></autopilot>)");
    aircraft = configuration.aircraft("6");
    check(aircraft.builtAirframe == built, "Built airframe must take precedence for parameters");
    parameters = configuration.parameters(aircraft);
    check(parameters.lowBatteryVoltage == 12 && parameters.criticalBatteryVoltage == 11 &&
          parameters.catastrophicBatteryVoltage == 10, "Built battery defines");
    check(parameters.cameraHorizontalFov == 80 && parameters.cameraVerticalFov == 40, "Camera FOV units");
    check(parameters.autopilotModes == std::vector<std::string>{"MAN", "Navigation"}, "Autopilot mode names");
    write(aircraft.signature, "ABCDEF0123456789ABCDEF0123456789\n");
    check(configuration.signature(aircraft) == "abcdef0123456789abcdef0123456789", "Signature normalization");
    write(aircraft.signature, "invalid");
    fails([&] { configuration.signature(aircraft); }, "Invalid signature accepted");

    // Replay files may appear and change after the live Configuration is created.
    const auto replayRoot = home / "var/replay";
    write(replayRoot / "conf/conf.xml",
      R"(<conf><aircraft ac_id="6" name="Replay" airframe="replay.xml" radio="radio.xml" gui_color="#123456"/></conf>)");
    const auto replay = configuration.aircraft("replay6");
    check(replay.root == replayRoot && replay.name == "Replay" && replay.replay &&
          replay.guiColor == "#123456", "Replay configuration isolation");
    write(replayRoot / "conf/conf.xml",
      R"(<conf><aircraft ac_id="6" name="UpdatedReplay" airframe="other.xml"/></conf>)");
    check(configuration.aircraft("replay6").name == "UpdatedReplay", "Replay configuration not refreshed");
    check(configuration.aircraft("6").name == "Test" && replay.name == "Replay", "Lookup results must own their data");
    fails([&] { configuration.aircraft("replay99"); }, "Unknown replay aircraft accepted");

    write(aircraft.radio, "<radio/>");
    write(aircraft.settings, "<settings/>");
    write(home / "var/messages.xml", "<protocol/>");
    configuration.writeSnapshot(home / "flight.log", home / "var/messages.xml", "flight.data", 1000);
    tinyxml2::XMLDocument snapshot;
    check(snapshot.LoadFile((home / "flight.log").c_str()) == tinyxml2::XML_SUCCESS, "Snapshot not readable");
    const auto* root = snapshot.RootElement();
    check(root->DoubleAttribute("time_of_day") == 1000 && std::string(root->Attribute("data_file")) == "flight.data",
          "Snapshot metadata");
    const auto* saved = root->FirstChildElement("conf")->FirstChildElement("aircraft");
    check(saved->FirstChildElement("dump") && saved->FirstChildElement("generated_settings") &&
          saved->FirstChildElement("radio") && root->FirstChildElement("protocol"), "Snapshot replay documents");
    // Snapshots keep source XML, while telemetry defaults prefer the built copy.
    check(saved->FirstChildElement("airframe")->FirstChildElement("autopilot") == nullptr, "Snapshot used built airframe");
    write(aircraft.flightPlan, "<dump/>");
    fails([&] { configuration.parameters(aircraft); }, "Missing flight_plan accepted");
    std::cout << "Configuration tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
