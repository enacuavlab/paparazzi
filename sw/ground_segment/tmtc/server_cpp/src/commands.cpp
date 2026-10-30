// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "server_helpers.h"
#include "legacy_text.h"
#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace server {
namespace {
int infoMessageByte(std::string_view value, const char* field) {
  auto digits = value;
  if (digits.starts_with('+'))
    digits.remove_prefix(1);
  const bool hexadecimal = digits.starts_with("0x") || digits.starts_with("0X");
  if (hexadecimal)
    digits.remove_prefix(2);
  unsigned int number = 0;
  const auto [end, error] = std::from_chars(digits.data(), digits.data() + digits.size(),
                                           number, hexadecimal ? 16 : 10);
  if (error != std::errc{} || end != digits.data() + digits.size() || number > 255)
    throw std::runtime_error("Invalid INFO_MSG_GROUND " + std::string(field) + " '" +
                             std::string(value) + "': expected an integer from 0 to 255");
  return static_cast<int>(number);
}
}

void Server::handleCommand(const Message& message, double now) {
  const auto& definition = message.getDefinition();
  const auto& name = definition.getName();
  if (name == "AIRCRAFT_DIE") {
    aircraft_.erase(message.getField<std::string>("ac_id"));
    return;
  }
  if (name == "INTRUDER") {
    validateFiniteFields(message);
    intruders_.insert_or_assign(message.getField<std::string>("id"), TrafficReport{
      .position = {message.getFieldAs<double>("lat") / 1e7, message.getFieldAs<double>("lon") / 1e7, message.getFieldAs<double>("alt") / 1000},
      .course = message.getFieldAs<double>("course") * geo::rad,
      .speed = message.getFieldAs<double>("speed"),
      .climbRate = message.getFieldAs<double>("climb"),
      .timeOfWeek = message.getFieldAs<std::uint32_t>("itow"),
      .updatedAt = now
    });
    logMessage("ground", message, now - startedAt_);
    return;
  }
  if (name == "LINK_REPORT") {
    if (auto it = aircraft_.find(message.getField<std::string>("ac_id")); it != aircraft_.end()) {
      validateFiniteFields(message);
      it->second.links.insert_or_assign(message.getField<std::string>("link_id"), LinkStatus{
        .updatedAt = now,
        .rxLostTime = message.getFieldAs<std::uint32_t>("rx_lost_time"),
        .rxBytes = message.getFieldAs<std::uint32_t>("rx_bytes"),
        .rxMessages = message.getFieldAs<std::uint32_t>("rx_msgs"),
        .rxBytesRate = message.getFieldAs<double>("rx_bytes_rate"),
        .txMessages = message.getFieldAs<std::uint32_t>("tx_msgs"),
        .pingTime = message.getFieldAs<double>("ping_time")
      });
      logMessage(it->first, message, now - startedAt_);
    }
    return;
  }
  if (name == "WIND_CLEAR") {
    if (auto it = aircraft_.find(message.getField<std::string>("ac_id")); it != aircraft_.end())
      it->second.wind = {};
    return;
  }
  static const std::map<std::string, std::string> commands = {
    {"DL_SETTING", "SETTING"}, {"GET_DL_SETTING", "GET_SETTING"}, {"JUMP_TO_BLOCK", "BLOCK"}, {"DL_EMERGENCY_CMD", "EMERGENCY_CMD"}
  };
  Message out;
  std::string id;
  if (auto cmd = commands.find(name); cmd != commands.end()) {
    id = message.getField<std::string>("ac_id");
    out = makeMessage(cmd->second, "ground_dl");
    for (size_t j = 0; j < out.getDefinition().getNbFields(); ++j) {
      auto key = out.getFieldDefinition(j).getName();
      if (key == "ac_id")
        out.setField(key, aircraftId(id));
      else
        copyField(out, message, key, key);
    }
  } else if (name == "MOVE_WAYPOINT") {
    id = message.getField<std::string>("ac_id");
    out = makeMessage("MOVE_WP", "ground_dl").setField(
      "ac_id", aircraftId(id),
      "wp_id", message.getFieldAs<double>("wp_id"),
      "lat", std::round(message.getFieldAs<double>("lat") * 1e7),
      "lon", std::round(message.getFieldAs<double>("long") * 1e7),
      "alt", std::round(message.getFieldAs<double>("alt") * 1000)
    );
  } else if (name == "INFO_MSG_GROUND") {
    auto dest = message.getField<std::string>("dest");
    auto colon = dest.find(':');
    id = dest.substr(0, colon);
    // Ground-agent names are valid destinations, but need no datalink uplink.
    // Numeric-looking destinations must be fully parsed, including hex broadcast.
    if (!id.empty() && !std::isdigit(static_cast<unsigned char>(id.front())) &&
        id.front() != '+' && id.front() != '-')
      return;
    const int destination = infoMessageByte(id, "destination");
    const int fd = colon == std::string::npos ? 0 : infoMessageByte(dest.substr(colon + 1), "fd");
    out = makeMessage("INFO_MSG_UP", "ground_dl").setField(
      "ac_id", destination,
      "fd", fd,
      "msg", message.getField<std::string>("msg")
    );
  } else if (name == "RAW_DATALINK") {
    id = message.getField<std::string>("ac_id");
    auto body = message.getField<std::string>("message");
    std::replace(body.begin(), body.end(), ';', ' ');
    auto def = dictionary_.getDefinition(body.substr(0, body.find(' ')));
    if (def.getClassId() != dictionary_.getClassId("datalink"))
      throw std::runtime_error("RAW_DATALINK must contain a datalink message");
    out = parseLegacyMessageBody(def, "ground_dl", body);
  } else
    return;
  validateFiniteFields(out);
  send_(out);
  logMessage(id, out, now - startedAt_);
  if (name == "DL_SETTING" || name == "GET_DL_SETTING") {
    int index = static_cast<int>(message.getFieldAs<double>("index"));
    if (auto it = aircraft_.find(id); it != aircraft_.end() && index >= 0 && index < 256)
      it->second.settings[index] = std::nullopt;
  }
}
}
