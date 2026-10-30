// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <filesystem>
#include <string>
namespace server {
struct Options {
  std::filesystem::path home, messages;
  std::string bus = "127.255.255.255:2010", hostname = "localhost", logName;
  std::string jsonAddress = "127.0.0.1";
  int port = 8889, jsonPort = 9870;
  bool logging = true, http = false, noMd5 = false;
  bool replayOld = false, json = true, verbose = false;
};
}
