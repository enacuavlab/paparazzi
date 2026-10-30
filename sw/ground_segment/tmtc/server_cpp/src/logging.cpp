// SPDX-License-Identifier: GPL-2.0-or-later
#include "server.h"
#include "legacy_text.h"
#include <iomanip>

namespace server {
void Server::logMessage(const std::string& id, const Message& m, double time) {
  if (!dataLog_)
    return;
  auto line = serializeLegacyMessage(m);
  line = line.substr(line.find(' ') + 1);
  *dataLog_ << std::fixed << std::setprecision(3) << time << ' ' << id << ' ' << line << '\n';
  dataLog_->flush();
}
}
