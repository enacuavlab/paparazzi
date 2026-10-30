// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <pprzlink/Message.h>
namespace server {
// Compatibility grammar for .data logs and RAW_DATALINK, independent of transport.
std::string serializeLegacyMessage(const pprzlink::Message& message);
pprzlink::Message parseLegacyMessageBody(const pprzlink::MessageDefinition& definition,
                                        const std::string& sender, const std::string& body);
}
