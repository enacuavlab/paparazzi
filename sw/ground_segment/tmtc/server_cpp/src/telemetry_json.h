// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <pprzlink/Message.h>

namespace server {
std::string serializeTelemetryJson(const pprzlink::Message& message, const std::string& name, double timestamp);
}
