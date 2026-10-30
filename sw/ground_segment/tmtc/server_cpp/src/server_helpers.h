// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include "server.h"
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>

namespace server {
inline std::string modeName(int i, std::initializer_list<const char*> names) {
  return i >= 0 && static_cast<size_t>(i) < names.size() ? *(names.begin() + i) : "UNK";
}
inline int aircraftId(const std::string& id) {
  size_t end = 0;
  int value = std::stoi(id, &end);
  if (end != id.size() || value < 0 || value > 255)
    throw std::runtime_error("Invalid aircraft ID: " + id);
  return value;
}
inline void copyField(Message& to, const Message& from, const std::string& dest, const std::string& source) {
  std::visit([&](const auto & v) {
    to.setField(dest, v);
  }, from.getField(source));
}
inline void validateFiniteFields(const Message& m) {
  for (size_t i = 0; i < m.getDefinition().getNbFields(); ++i)
    std::visit([](const auto & v) {
    using T = std::decay_t<decltype(v)>;
    if constexpr(std::is_floating_point_v<T>) {
      if (!std::isfinite(v))
        throw std::runtime_error("Non-finite telemetry value");
    } else if constexpr(!std::is_arithmetic_v<T> && !std::is_same_v<T, std::string>) {
      if constexpr(std::is_floating_point_v<typename T::value_type>)
        for (auto x : v)
          if (!std::isfinite(x))
            throw std::runtime_error("Non-finite telemetry array");
    }
  }, m.getField(i));
}
} // namespace server
