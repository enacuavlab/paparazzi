// SPDX-License-Identifier: GPL-2.0-or-later
#include "application.h"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <iostream>
#include <utility>

namespace {
void configureOptions(CLI::App& app, server::Options& options) {
  options.home = std::filesystem::current_path();
  app.add_option("--home", options.home, "Paparazzi home directory")
    ->envname("PAPARAZZI_HOME")->check(CLI::ExistingDirectory)->capture_default_str();
  app.add_option("--messages", options.messages, "Message definitions (default: HOME/var/messages.xml)")
    ->check(CLI::ExistingFile);
  app.add_option("-b,--bus", options.bus, "Ivy bus")->envname("IVYBUS")->capture_default_str();
  app.add_flag("-v,--verbose", options.verbose, "Trace startup, bindings, messages and aircraft state to stderr");
  app.add_flag_callback("-n,--no-log", [&options] { options.logging = false; }, "Disable flight logging");
  app.add_option("--log-name", options.logName, "Flight log filename prefix")
  ->check([](const std::string& value) {
    return value.find_first_of("/\\") == std::string::npos ? std::string{} : "Log name must be a filename prefix"
      ;
  });
  app.add_flag("--no-md5-check", options.noMd5, "Skip live configuration signature checks");
  app.add_flag("--replay-old-log", options.replayOld, "Register old replay streams on PPRZ_MODE");
  app.add_flag("--http", options.http, "Return HTTP configuration URLs");
  app.add_option("--hostname", options.hostname, "External configuration HTTP server host")
    ->capture_default_str();
  app.add_option("--port", options.port, "External configuration HTTP server port")
    ->check(CLI::Range(1, 65535))->capture_default_str();
  app.add_flag_callback("--udp-json-stream-disable", [&options] { options.json = false; },
    "Disable UDP JSON telemetry");
  app.add_option("--udp-json-stream-addr", options.jsonAddress, "UDP JSON destination IPv4 address")
    ->capture_default_str();
  app.add_option("--udp-json-stream-port", options.jsonPort, "UDP JSON destination port")
    ->check(CLI::Range(1, 65535))->capture_default_str();
}
} // namespace

int main(int argc, char** argv) {
  try {
    server::Options options;
    CLI::App app{"Paparazzi ground telemetry server", "server_cpp"};
    configureOptions(app, options);
    try {
      app.parse(argc, argv);
    } catch (const CLI::ParseError& error) {
      return app.exit(error);
    }
    options.home = std::filesystem::absolute(options.home);
    if (options.messages.empty())
      options.messages = options.home / "var/messages.xml";
    return server::runServer(std::move(options));
  } catch (const std::exception& error) {
    std::cerr << "server_cpp: " << error.what() << '\n';
    return 1;
  }
}
