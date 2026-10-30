// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include <pprzlink/IvyMessageCodec.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unistd.h>

using namespace server;
namespace fs = std::filesystem;
namespace {
double number(const Message& message, const std::string& field) {
  return message.getFieldAs<double>(field);
}
std::string text(const Message& message, const std::string& field) {
  return message.getField<std::string>(field);
}
void check(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance = 1e-5) {
  check(std::abs(actual - expected) < tolerance,
        "Expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
struct Fixture {
  fs::path home;
  explicit Fixture(const fs::path& source) {
    char path[] = "/tmp/server-cpp-test-XXXXXX";
    char *p = mkdtemp(path);
    if (!p)
      throw std::runtime_error("mkdtemp failed");
    home = p;
    fs::create_directories(home / "conf");
    fs::create_directories(home / "var");
    fs::copy_file(source / "sw/ext/pprzlink/message_definitions/v1.0/messages.xml", home / "var/messages.xml");
    std::ofstream(home / "conf/conf.xml") <<
                                          R"(<conf><aircraft ac_id="6" name="Test" airframe="airframe.xml" radio="radio.xml" gui_color="#123456"/><aircraft ac_id="7" name="Other" airframe="airframe.xml" radio="radio.xml"/></conf>)";
    std::ofstream(home / "conf/airframe.xml") <<
        R"(<airframe><section name="BAT"><define name="LOW_BAT_LEVEL" value="11.5"/></section></airframe>)";
    std::ofstream(home / "conf/radio.xml") << "<radio/>";
    for (auto name : {
           "Test", "Other"
         }) {
      auto dir = home / "var/aircrafts" / name;
      fs::create_directories(dir / "conf");
      std::ofstream(dir / "flight_plan.xml") <<
                                             R"(<dump><flight_plan name="test" lat0="43" lon0="1" ground_alt="100"/></dump>)";
      std::ofstream(dir / "settings.xml") << "<settings/>";
      std::ofstream(dir / "conf/aircraft.md5") << "0102030405060708090a0b0c0d0e0f10\n";
    }
  }
  ~Fixture() {
    fs::remove_all(home);
  }
};
Message populated(const pprzlink::MessageDictionary& dict, const std::string& name) {
  Message m(dict.getDefinition(name));
  m.setSenderId(std::string("test"));
  for (size_t j = 0; j < m.getDefinition().getNbFields(); ++j) {
    auto field = m.getFieldDefinition(j);
    auto type = field.getType();
    auto key = field.getName();
    if (type.isArray())
      m.setField(key, std::vector<uint8_t>(type.getArraySize()));
    else if (type.getBaseType() == pprzlink::BaseType::STRING)
      m.setField(key, std::string(""));
    else
      m.setField(key, 0);
  }
  return m;
}
const Message& last(const std::vector<Message>& sent, const std::string& name, const std::string& id = "6") {
  for (auto it = sent.rbegin(); it != sent.rend(); ++it)
    if (it->getDefinition().getName() == name && (!it->getDefinition().hasFieldName("ac_id") ||
        text(*it, "ac_id") == id))
      return *it;
  throw std::runtime_error("No " + name + " for " + id);
}
}
int main(int argc, char** argv) {
  try {
    check(argc == 2, "Expected repository path");
    Fixture fixture(argv[1]);
    Options o;
    o.home = fixture.home;
    o.messages = o.home / "var/messages.xml";
    const pprzlink::MessageDictionary dict(o.messages.string());
    const paparazzi::Configuration configuration(o.home);
    std::vector<Message> sent;
    std::ostringstream log;
    std::string stream;
    Server s(o, configuration, dict, [&](const Message & m) {
      // Validate that every publication is complete and serializable.
      pprzlink::ivy_codec::serializeLegacyMessage(m);
      sent.push_back(m);
    }, &log, 1000);
    s.streamJson = [&](const std::string & value) {
      stream = value;
    };
    auto alive = populated(dict, "ALIVE");
    // Wrong signatures must not register aircraft. A later correct ALIVE can retry.
    alive.setField("md5sum", std::vector<uint8_t>(16, 42));
    s.receive("6", alive, 1001);
    check(s.aircraft().empty(), "MD5 mismatch registered aircraft");
    std::vector<uint8_t> digest;
    for (int j = 1; j <= 16; ++j)
      digest.push_back(j);
    alive.setField("md5sum", digest);
    s.receive("6", alive, 1001);
    check(s.aircraft().size() == 1, "ALIVE did not register aircraft");
    check(text(last(sent, "NEW_AIRCRAFT"), "ac_id") == "6", "Missing discovery");
    s.receive("6", alive, 1002);
    check(s.aircraft().size() == 1, "Duplicate registration");
    auto req = populated(dict, "AIRCRAFTS_REQ");
    check(text(s.answer(req), "ac_list") == "6,", "Aircraft list");
    req = populated(dict, "CONFIG_REQ");
    req.setField("ac_id", std::string("6"));
    auto config = s.answer(req);
    check(text(config, "ac_name") == "Test", "Configuration name");
    check(text(config, "flight_plan") == "file://" + o.home.string() + "/var/aircrafts/Test/flight_plan.xml",
          "Configuration URL");
    auto gps = populated(dict, "GPS_LLA");
    gps.setField("lat", 430000000, "lon", 10000000, "speed", 1234, "course", 900, "mode", 3, "itow", 123000);
    s.receive("6", gps, 1003);
    near(s.aircraft().at("6").groundSpeed, 12.34);
    near(s.aircraft().at("6").course, geo::pi / 2);
    check(stream.find("\"timestamp\":3") != std::string::npos, "JSON timestamp");
    check(log.str().find("3.000 6 GPS_LLA") != std::string::npos, "Reception time not recorded");
    auto attitude = populated(dict, "ATTITUDE");
    attitude.setField("phi", .1f, "theta", -.2f, "psi", 1.f);
    s.receive("6", attitude, 1004);
    auto est = populated(dict, "ESTIMATOR");
    est.setField("z", 150.f, "z_dot", 2.f);
    s.receive("6", est, 1004);
    auto energy = populated(dict, "ENERGY");
    energy.setField("voltage", 11.f, "current", 5.f, "charge", .5f, "throttle", 42.f);
    s.receive("6", energy, 1004);
    auto navref = populated(dict, "NAVIGATION_REF");
    navref.setField("utm_east", 400000.f, "utm_north", 4800000.f, "utm_zone", 31, "ground_alt", 100.f);
    s.receive("6", navref, 1004);
    auto desired = populated(dict, "DESIRED");
    desired.setField("x", 100.f, "y", 200.f, "altitude", 200.f, "climb", 3.f);
    s.receive("6", desired, 1004);
    s.broadcast(1005);
    near(number(last(sent, "FLIGHT_PARAM"), "lat"), 43);
    near(number(last(sent, "FLIGHT_PARAM"), "alt"), 150);
    near(number(last(sent, "FLIGHT_PARAM"), "agl"), 50);
    near(number(last(sent, "FLIGHT_PARAM"), "roll"), .1 / geo::rad);
    near(number(last(sent, "ENGINE_STATUS"), "bat"), 11);
    near(number(last(sent, "NAV_STATUS"), "target_alt"), 200);
    near(number(last(sent, "TELEMETRY_STATUS"), "time_since_last_msg"), 1);
    s.checkAlerts(1005);
    check(text(last(sent, "BAT_LOW"), "level") == "WARNING", "Configured battery threshold not used");
    auto move = populated(dict, "MOVE_WAYPOINT");
    move.setField("ac_id", std::string("6"), "wp_id", 2, "lat", -43.1234567, "long", 1.2345678, "alt", 123.456f);
    s.receive("gcs", move, 1006);
    check(sent.back().getDefinition().getName() == "MOVE_WP", "Waypoint command not forwarded");
    near(number(sent.back(), "lat"), -431234567, .1);
    near(number(sent.back(), "alt"), 123456, .1);
    auto setting = populated(dict, "DL_SETTING");
    setting.setField("ac_id", std::string("6"), "index", 2, "value", 1.25f);
    s.receive("gcs", setting, 1006);
    check(sent.back().getDefinition().getName() == "SETTING", "Setting command not forwarded");
    s.broadcast(1006);
    check(text(last(sent, "DL_VALUES"), "values") == "?,?,?,", "Unconfirmed setting should be unknown");
    auto value = populated(dict, "DL_VALUE");
    value.setField("index", 2, "value", 1.25f);
    s.receive("6", value, 1007);
    s.broadcast(1007);
    check(text(last(sent, "DL_VALUES"), "values") == "?,?,1.25,", "Setting feedback missing");
    auto jump = populated(dict, "JUMP_TO_BLOCK");
    jump.setField("ac_id", std::string("6"), "block_id", 4);
    s.receive("gcs", jump, 1007);
    check(sent.back().getDefinition().getName() == "BLOCK", "Block command missing");
    near(number(sent.back(), "block_id"), 4);
    auto raw = populated(dict, "RAW_DATALINK");
    raw.setField("ac_id", std::string("6"), "message", std::string("BLOCK;5;6"));
    s.receive("gcs", raw, 1008);
    check(sent.back().getDefinition().getName() == "BLOCK", "Raw uplink missing");
    near(number(sent.back(), "block_id"), 5);
    auto info = populated(dict, "INFO_MSG_GROUND");
    info.setField("source", std::string("gcs"), "msg", std::string("hello aircraft"));
    for (const auto* destination : {"azerty", "ground_agent", "azerty:2"}) {
      info.setField("dest", std::string(destination));
      const auto sentBefore = sent.size();
      const auto logBefore = log.str();
      s.receive("gcs", info, 1008);
      check(sent.size() == sentBefore && log.str() == logBefore,
            "Ground-agent INFO_MSG_GROUND was forwarded or logged");
    }
    struct InfoDestination { const char* text; int aircraftId; int fd; };
    for (const auto& destination : {
        InfoDestination{"6", 6, 0}, {"6:2", 6, 2}, {"0", 0, 0},
        {"0xFF", 255, 0}, {"0Xff:0x02", 255, 2}, {"008:01", 8, 1},
        {"255:255", 255, 255}}) {
      info.setField("dest", std::string(destination.text));
      const auto sentBefore = sent.size();
      s.receive("gcs", info, 1008);
      check(sent.size() == sentBefore + 1 && sent.back().getDefinition().getName() == "INFO_MSG_UP",
            "Numeric INFO_MSG_GROUND not forwarded");
      check(std::get<std::string>(sent.back().getSenderId()) == "ground_dl", "Info uplink sender");
      near(number(sent.back(), "ac_id"), destination.aircraftId);
      near(number(sent.back(), "fd"), destination.fd);
      const auto& payload = sent.back().getField<std::vector<char>>("msg");
      check(std::string(payload.begin(), payload.end()) == "hello aircraft", "Info payload changed");
    }
    for (const auto* destination : {"", ":2", "6:", "6:foo", "6:2junk", "6:2:3", "6:-1",
                                   "6:256", "-1", "256", "0x100", "0x", "6junk", "0xGG",
                                   "999999999999999999999999999", "6:1.5"}) {
      info.setField("dest", std::string(destination));
      const auto sentBefore = sent.size();
      const auto logBefore = log.str();
      bool rejected = false;
      try {
        s.receive("gcs", info, 1008);
      } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()).find("INFO_MSG_GROUND") != std::string::npos;
      }
      check(rejected && sent.size() == sentBefore && log.str() == logBefore,
            "Invalid INFO_MSG_GROUND destination accepted: " + std::string(destination));
    }
    auto report = populated(dict, "LINK_REPORT");
    auto datalink = populated(dict, "DATALINK_REPORT");
    datalink.setField("uplink_lost_time", 4, "uplink_nb_msgs", 55,
                      "downlink_nb_msgs", 66, "downlink_rate", 128);
    s.receive("6", datalink, 1008);
    report.setField("ac_id", std::string("6"), "link_id", std::string("2"), "rx_bytes", 100, "rx_msgs", 10,
                    "rx_lost_time", 3);
    s.receive("link", report, 1008);
    s.broadcast(1010);
    auto status = last(sent, "TELEMETRY_STATUS");
    check(text(status, "link_id") == "2", "Link ID missing");
    near(number(status, "rx_bytes"), 100);
    near(number(status, "time_since_last_msg"), 5);
    near(number(status, "uplink_lost_time"), 4);
    near(number(status, "uplink_msgs"), 55);
    near(number(status, "downlink_msgs"), 66);
    near(number(status, "downlink_rate"), 128);
    auto wp = populated(dict, "WP_MOVED_LLA");
    wp.setField("wp_id", 3, "lat", 430000000, "lon", 10000000, "alt", 200000);
    s.receive("6", wp, 1010);
    s.broadcast(1010);
    near(number(last(sent, "WAYPOINT_MOVED"), "alt"), 200);
    // ECEF origin at the equator: ENU up must increase altitude, east longitude.
    auto ref = populated(dict, "INS_REF");
    ref.setField("ecef_x0", 637813700, "alt0", 0, "hmsl0", -30000);
    s.receive("6", ref, 1011);
    auto rotor = populated(dict, "ROTORCRAFT_FP");
    rotor.setField("east", 25600, "north", 0, "up", 2560, "veast", 5242880, "vup", 524288, "thrust", 4800, "phi",
                   4096);
    s.receive("6", rotor, 1011);
    s.broadcast(1011);
    auto flight = last(sent, "FLIGHT_PARAM");
    near(number(flight, "long"), .000898315, 1e-7);
    near(number(flight, "alt"), -20, 1e-2);
    near(number(flight, "speed"), 10);
    near(number(flight, "climb"), 1);
    near(number(flight, "roll"), 1 / geo::rad, 1e-4);
    near(number(last(sent, "ENGINE_STATUS"), "throttle"), 50);
    // Non-finite telemetry is reported without corrupting existing state.
    attitude.setField("phi", std::numeric_limits<float>::quiet_NaN());
    double previous = s.aircraft().at("6").roll;
    s.receive("6", attitude, 1012);
    check(sent.back().getDefinition().getName() == "TELEMETRY_ERROR", "Nonfinite telemetry accepted");
    near(s.aircraft().at("6").roll, previous);
    // Multiple aircraft publish traffic; removed aircraft stop broadcasting and can return.
    s.receive("7", alive, 1013);
    s.receive("7", gps, 1013);
    sent.clear();
    s.broadcast(1013);
    bool traffic = false;
    for (auto& m : sent)
      if (m.getDefinition().getName() == "ACINFO" || m.getDefinition().getName() == "ACINFO_LLA")
        traffic = true;
    check(traffic, "Multi-aircraft traffic missing");
    auto die = populated(dict, "AIRCRAFT_DIE");
    die.setField("ac_id", std::string("6"));
    s.receive("gcs", die, 1014);
    sent.clear();
    s.broadcast(1014);
    for (auto& m : sent)
      if (m.getDefinition().getClassId() == dict.getClassId("ground"))
        check(text(m, "ac_id") != "6", "Removed aircraft still broadcasts");
    s.receive("6", alive, 1015);
    check(s.aircraft().size() == 2, "Aircraft could not re-register");

    // Intruders expire, so stale ADS-B traffic cannot keep being rebroadcast.
    auto intruder = populated(dict, "INTRUDER");
    intruder.setField("id", std::string("123"), "name", std::string("external"),
                      "lat", 430000000, "lon", 10000000, "alt", 100000,
                      "course", 90.f, "speed", 10.f);
    s.receive("adsb", intruder, 1020);
    sent.clear();
    s.checkAlerts(1021);
    bool external = false;
    for (const auto& m : sent)
      if (m.getDefinition().getName() == "ACINFO" && number(m, "ac_id") == 223) external = true;
    check(external, "Intruder traffic missing");
    sent.clear();
    s.checkAlerts(1031);
    for (const auto& m : sent)
      check(m.getDefinition().getName() != "ACINFO", "Expired intruder still broadcasts");

    // Four ground-speed vectors on a circle imply an eastward 5 m/s wind.
    Server windServer(o, configuration, dict, [&](const Message& m) { sent.push_back(m); });
    windServer.receive("6", alive, 2000);
    auto modes = populated(dict, "PPRZ_MODE");
    modes.setField("ap_mode", 2, "mcu1_status", 4);
    windServer.receive("6", modes, 2000);
    for (auto velocity : std::array<std::array<double, 2>, 4>{{{15, 0}, {5, 10}, {-5, 0}, {5, -10}}}) {
      gps.setField("speed", std::round(std::hypot(velocity[0], velocity[1])*100));
      gps.setField("course", std::round(geo::normalize(std::atan2(velocity[0], velocity[1]))/geo::rad*10));
      windServer.receive("6", gps, 2001);
    }
    windServer.publishWind();
    near(number(last(sent, "WIND"), "wspeed"), 5, .02);
    near(number(last(sent, "WIND"), "mean_aspeed"), 10, .02);
    near(number(last(sent, "WIND"), "dir"), 270, .1);
    auto clear = populated(dict, "WIND_CLEAR");
    clear.setField("ac_id", std::string("6"));
    windServer.receive("gcs", clear, 2002);
    sent.clear();
    windServer.publishWind();
    check(sent.empty(), "WIND_CLEAR left old samples");

    // Nearby aircraft on opposite sides of the dateline still need an alert.
    Server proximityServer(o, configuration, dict, [&](const Message& message) { sent.push_back(message); });
    proximityServer.receive("6", alive, 3000);
    proximityServer.receive("7", alive, 3000);
    auto trafficPosition = populated(dict, "GPS_LLA");
    trafficPosition.setField("lat", 0, "lon", 1799998000, "mode", 3, "speed", 1000, "course", 900);
    proximityServer.receive("6", trafficPosition, 3000);
    trafficPosition.setField("lon", -1799998000, "course", 2700);
    proximityServer.receive("7", trafficPosition, 3000);
    proximityServer.checkAlerts(3001);
    check(text(last(sent, "AIR_PROX", "6,7"), "level") == "CRITICAL", "Dateline approach was missed");
    trafficPosition.setField("lon", 1799998000, "course", 2700);
    proximityServer.receive("6", trafficPosition, 3001);
    trafficPosition.setField("lon", -1799998000, "course", 900);
    proximityServer.receive("7", trafficPosition, 3001);
    proximityServer.checkAlerts(3002);
    check(text(last(sent, "AIR_PROX", "6,7"), "level") == "WARNING", "Departing traffic marked critical");

    // Replay uses its own configuration tree and skips live MD5 checks.
    fs::create_directories(o.home / "var/replay");
    fs::copy(o.home / "conf", o.home / "var/replay/conf", fs::copy_options::recursive);
    fs::create_directories(o.home / "var/replay/var");
    fs::copy(o.home / "var/aircrafts", o.home / "var/replay/var/aircrafts", fs::copy_options::recursive);
    s.receive("replay6", alive, 1040);
    check(s.aircraft().contains("replay6"), "Replay aircraft not registered");
    req.setField("ac_id", std::string("replay6"));
    check(text(s.answer(req), "flight_plan").find("/var/replay/var/aircrafts/Test/") != std::string::npos,
          "Replay URL does not use replay configuration");
    Options httpOptions = o;
    httpOptions.http = true;
    httpOptions.hostname = "config.example";
    httpOptions.port = 8080;
    Server httpServer(httpOptions, configuration, dict, [](const Message&) {});
    check(text(httpServer.answer(req), "flight_plan") ==
          "http://config.example:8080/var/replay/var/aircrafts/Test/flight_plan.xml", "Replay HTTP URL");
    req.setField("ac_id", std::string("6"));
    const auto httpConfig = httpServer.answer(req);
    check(text(httpConfig, "airframe") == "http://config.example:8080/conf/airframe.xml", "Source airframe HTTP URL");
    check(text(httpConfig, "settings") == "http://config.example:8080/var/aircrafts/Test/settings.xml", "Settings HTTP URL");
    // UTM conversions must preserve both hemispheres and zone-boundary longitudes.
    for (geo::Point p : std::vector<geo::Point> {{43, 1, 0}, {-33, 18, 0}, {0, -177, 0}, {60, 179, 0}}) {
      auto utm = geo::toUtm(p);
      auto q = geo::fromUtm(utm.east, utm.north, utm.zone);
      near(q.lat, p.lat, 1e-6);
      near(q.lon, p.lon, 1e-6);
    }
    configuration.writeSnapshot(o.home / "flight.log", o.messages, "flight.data", 1000);
    tinyxml2::XMLDocument saved;
    check(saved.LoadFile((o.home / "flight.log").c_str()) == tinyxml2::XML_SUCCESS, "Invalid snapshot XML");
    auto* savedAircraft = saved.RootElement()->FirstChildElement("conf")->FirstChildElement("aircraft");
    check(savedAircraft->FirstChildElement("dump"), "Replay snapshot lost the generated flight plan");
    check(savedAircraft->FirstChildElement("generated_settings"), "Replay snapshot lost generated settings");
    pprzlink::MessageDictionary snapshot((o.home / "flight.log").string());
    check(snapshot.getDefinition("FLIGHT_PARAM").getNbFields() ==
          dict.getDefinition("FLIGHT_PARAM").getNbFields(), "Snapshot cannot be read as replay schema");
    std::cout << "All server core checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test failed: " << e.what() << '\n';
    return 1;
  }
}
