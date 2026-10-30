// SPDX-License-Identifier: GPL-2.0-or-later
#include "telemetry_json.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <type_traits>

namespace server {
namespace {
std::string quote(const std::string& s) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    switch (c) {
    case '"':
      out << "\\\"";
      break;
    case '\\':
      out << "\\\\";
      break;
    case '\n':
      out << "\\n";
      break;
    case '\r':
      out << "\\r";
      break;
    case '\t':
      out << "\\t";
      break;
    default:
      if (c < 32)
        out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
      else
        out << c;
    }
  }
  out << '"';
  return out.str();
}
template<class T> void jsonValue(std::ostream& o, const T& v) {
  if constexpr(std::is_arithmetic_v<T>) {
    if constexpr(std::is_floating_point_v<T>) {
      if (!std::isfinite(v)) {
        o << "null";
        return;
      }
    }
    o << +v;
  } else if constexpr(std::is_same_v<T, std::string>)
    o << quote(v);
  else {
    o << '[';
    bool comma = false;
    for (const auto& x : v) {
      if (comma)
        o << ',';
      comma = true;
      jsonValue(o, x);
    }
    o << ']';
  }
}
}
std::string serializeTelemetryJson(const pprzlink::Message& m, const std::string& name, double timestamp) {
  std::ostringstream o;
  o << std::setprecision(17) << "{" << quote(name) << ":{" << quote(m.getDefinition().getName()) << ":{";
  for (size_t i = 0; i < m.getDefinition().getNbFields(); ++i) {
    if (i)
      o << ',';
    o << quote(m.getFieldDefinition(i).getName()) << ':';
    std::visit([&](const auto & v) {
      using T = std::decay_t<decltype(v)>;
      if constexpr(!std::is_arithmetic_v<T> && !std::is_same_v<T, std::string>) {
        o << '{';
        for (size_t j = 0; j < v.size(); ++j) {
          if (j)
            o << ',';
          o << quote(m.getFieldDefinition(i).getName() + "_" + std::to_string(j)) << ':';
          jsonValue(o, v[j]);
        }
        o << '}';
      } else
        jsonValue(o, v);
    }, m.getField(i));
  }
  o << "}},\"timestamp\":" << timestamp << '}';
  return o.str();
}
}
