// SPDX-License-Identifier: GPL-2.0-or-later
#include "legacy_text.h"
#include <pprzlink/IvyMessageCodec.h>
namespace server {
std::string serializeLegacyMessage(const pprzlink::Message& message) {
  return pprzlink::ivy_codec::serializeLegacyMessage(message);
}
pprzlink::Message parseLegacyMessageBody(const pprzlink::MessageDefinition& definition,
                                        const std::string& sender, const std::string& body) {
  return pprzlink::ivy_codec::parseLegacyMessageBody(definition, sender, body);
}
}
