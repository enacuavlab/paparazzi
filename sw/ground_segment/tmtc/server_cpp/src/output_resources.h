// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include "options.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <system_error>

namespace server {
class JsonSocket {
 public:
  explicit JsonSocket(const Options& options) {
    if (!options.json)
      return;
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(options.jsonPort);
    if (inet_pton(AF_INET, options.jsonAddress.c_str(), &destination_.sin_addr) != 1)
      throw std::runtime_error("Invalid JSON destination IPv4 address");
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
      throw std::system_error(errno, std::generic_category(), "JSON socket");
  }

  ~JsonSocket() {
    if (fd_ >= 0)
      close(fd_);
  }
  JsonSocket(const JsonSocket&) = delete;
  JsonSocket& operator=(const JsonSocket&) = delete;

  void send(const std::string& data) const {
    if (fd_ < 0)
      return;
    if (sendto(fd_, data.data(), data.size(), 0, reinterpret_cast<const sockaddr*>(&destination_),
      sizeof destination_) < 0)
      std::cerr << "UDP JSON: " << std::error_code(errno, std::generic_category()).message() << '\n';
  }

 private:
  int fd_ = -1;
  sockaddr_in destination_{};
};

class FlightLog {
 public:
  FlightLog(const Options& options, double startedAt) {
    if (!options.logging)
      return;
    const auto directory = options.home / "var/logs";
    std::filesystem::create_directories(directory);
    const auto epoch = static_cast<std::time_t>(startedAt);
    std::tm local{};
    localtime_r(&epoch, &local);
    char timestamp[32];
    std::strftime(timestamp, sizeof timestamp, "%y_%m_%d__%H_%M_%S", &local);
    const std::string prefix = options.logName.empty() ? timestamp : options.logName + "__" + timestamp;
    auto basename = prefix;
    for (int suffix = 1; std::filesystem::exists(directory / (basename + ".data")) ||
      std::filesystem::exists(directory / (basename + ".log")); ++suffix)
      basename = prefix + "_" + std::to_string(suffix);
    metadata_ = directory / (basename + ".log");
    dataPath_ = directory / (basename + ".data");
    data_.open(dataPath_);
    if (!data_)
      throw std::runtime_error("Cannot create data log: " + dataPath_.string());
  }

  std::ostream* stream() {
    return data_.is_open() ? &data_ : nullptr;
  }
  const std::filesystem::path& metadataPath() const {
    return metadata_;
  }
  const std::filesystem::path& dataPath() const {
    return dataPath_;
  }

 private:
  std::filesystem::path metadata_;
  std::filesystem::path dataPath_;
  std::ofstream data_;
};

}
