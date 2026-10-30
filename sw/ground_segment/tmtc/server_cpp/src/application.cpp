// SPDX-License-Identifier: GPL-2.0-or-later
#include "application.h"
#include "server.h"

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/steady_timer.hpp>
#include "pprz_connection.h"
#include "output_resources.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

namespace server {
namespace {
double currentTime() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// These resources borrow one dictionary and must be replaced together. Destroy
// the link before server/log/dictionary, and stop its worker before any of them.
struct Session {
  std::unique_ptr<pprzlink::MessageDictionary> dictionary;
  FlightLog log;
  Server server;
  PprzConnection link;

  Session(const Options& options, const paparazzi::Configuration& configuration,
          boost::asio::io_context& context, std::unique_ptr<pprzlink::MessageDictionary> definitions,
          std::function<void(const Message&)> send, double startedAt)
    : dictionary(std::move(definitions)), log(options, startedAt),
      server(options, configuration, *dictionary, std::move(send), log.stream(), startedAt),
      link(*dictionary, options.bus, context) {}
};

// The player rewrites these files when loading a log, even if its message
// definitions match the previous log. Check both to reset aircraft state too.
struct ReplayVersion {
  std::filesystem::file_time_type messages, configuration;
  bool operator==(const ReplayVersion&) const = default;
};

// All session changes and callbacks run on the application context. The native
// transport thread never accesses aircraft state or replaces the session.
class Application {
 public:
  explicit Application(Options options)
    : options_(std::move(options)), configuration_(options_.home), jsonSocket_(options_) {
    trace(options_.json ? "JSON destination=" + options_.jsonAddress + ":" + std::to_string(options_.jsonPort)
      : "JSON disabled");
    trace(options_.noMd5 ? "MD5 checks disabled" : "MD5 checks enabled");
    startSession(std::make_unique<pprzlink::MessageDictionary>(options_.messages.string()),
                 configuration_, options_.messages);
    startTimers();
  }

  ~Application() {
    context_.stop();
    session_.reset(); // Close/join the link while its callback targets still live.
  }

  void run() {
    signals_.async_wait([this](const boost::system::error_code& error, int) {
      if (!error)
        context_.stop();
    });
    trace("READY waiting for ALIVE" + std::string(options_.replayOld ? " or PPRZ_MODE" : ""));
    std::cout << "Paparazzi C++ server on " << options_.bus << '\n';
    session_->link.start();
    context_.run();
    session_->link.stop();
    trace("STOP rx=" + std::to_string(received_) + " tx=" + std::to_string(transmitted_));
  }

 private:
  Options options_;
  paparazzi::Configuration configuration_;
  boost::asio::io_context context_;
  boost::asio::signal_set signals_{context_, SIGINT, SIGTERM};
  boost::asio::steady_timer broadcastTimer_{context_};
  boost::asio::steady_timer alertTimer_{context_};
  boost::asio::steady_timer windTimer_{context_};
  JsonSocket jsonSocket_;
  std::unique_ptr<Session> session_;
  std::optional<ReplayVersion> replayVersion_;
  bool restartPending_ = false;
  std::size_t received_ = 0;
  std::size_t transmitted_ = 0;

  void trace(const std::string& message) const {
    if (options_.verbose)
      std::cerr << "[server_cpp] " << message << '\n';
  }

  void send(const Message& message) {
    session_->link.send(message);
    ++transmitted_;
    trace("TX " + message.toString());
  }

  ReplayVersion replayVersion() const {
    const auto root = options_.home / "var/replay";
    return {std::filesystem::last_write_time(root / "var/messages.xml"),
            std::filesystem::last_write_time(root / "conf/conf.xml")};
  }

  void receive(const std::string& sender, const Message& message) {
    if (restartPending_)
      return; // Old-session messages queued ahead of the deferred restart.
    try {
      const bool telemetry = message.getClassId() == session_->dictionary->getClassId("telemetry");
      const auto& name = message.getDefinition().getName();
      const bool discovery = name == "ALIVE" || (options_.replayOld && name == "PPRZ_MODE");
      if (telemetry && sender.starts_with("replay") && discovery) {
        const auto version = replayVersion();
        if (!replayVersion_ || *replayVersion_ != version) {
          restartPending_ = true;
          trace("REPLAY scheduling session restart for " + sender);
          // The SDK forbids destroying a link from its callback. The copied
          // discovery message owns its definition/fields and survives the switch.
          boost::asio::post(context_, [this, sender, message, version] {
            restartReplay(sender, message, version);
          });
          return;
        }
      }
      if (replayVersion_ && telemetry && !sender.starts_with("replay"))
        return; // A replay session uses only the archived telemetry definitions.
      ++received_;
      trace("RX " + sender + " " + message.toString());
      session_->server.receive(sender, message, currentTime());
    } catch (const std::exception& error) {
      std::cerr << "Message from " << sender << ": " << error.what() << '\n';
    }
  }

  void startSession(std::unique_ptr<pprzlink::MessageDictionary> dictionary,
                    const paparazzi::Configuration& snapshotConfiguration,
                    const std::filesystem::path& messages) {
    const auto startedAt = currentTime();
    session_ = std::make_unique<Session>(options_, configuration_, context_, std::move(dictionary),
      [this](const Message& message) { send(message); }, startedAt);
    if (options_.logging) {
      // Each dictionary gets its own log and matching replay metadata.
      snapshotConfiguration.writeSnapshot(session_->log.metadataPath(), messages,
        session_->log.dataPath().filename().string(), startedAt);
      trace("LOG metadata=" + session_->log.metadataPath().string() +
            " data=" + session_->log.dataPath().string());
    } else {
      trace("LOG disabled");
    }
    if (options_.json)
      session_->server.streamJson = [this](const std::string& data) { jsonSocket_.send(data); };
    trace("BIND all decoded messages");
    session_->link.bind([this](const std::string& sender, const Message& message) {
      receive(sender, message);
    });
    for (const auto* name : {"AIRCRAFTS_REQ", "CONFIG_REQ"}) {
      trace("BIND request answerer " + std::string(name));
      session_->link.bindRequest(name, [this](const Message& request) {
        const auto& sender = std::get<std::string>(request.getSenderId());
        ++received_;
        trace("RX REQUEST " + sender + " " + request.toString());
        auto reply = session_->server.answer(request);
        ++transmitted_;
        trace("TX RESPONSE " + reply.toString());
        return reply;
      });
    }
  }

  void restartReplay(const std::string& sender, const Message& discovery, ReplayVersion version) {
    const auto root = options_.home / "var/replay";
    const auto messages = root / "var/messages.xml";
    std::unique_ptr<pprzlink::MessageDictionary> dictionary;
    std::unique_ptr<paparazzi::Configuration> snapshotConfiguration;
    try {
      // Validate files before closing the working session. A later ALIVE can
      // retry if a file was missing, incomplete or invalid during preparation.
      trace("LOAD replay messages " + messages.string());
      dictionary = std::make_unique<pprzlink::MessageDictionary>(messages.string());
      snapshotConfiguration = std::make_unique<paparazzi::Configuration>(root);
      snapshotConfiguration->aircraft(sender.substr(6));
      (void)dictionary->getDefinition(discovery.getDefinition().getName());
      for (const auto* name : {"AIRCRAFTS_REQ", "AIRCRAFTS", "CONFIG_REQ", "CONFIG"})
        (void)dictionary->getDefinition(name);
      if (replayVersion() != version)
        throw std::runtime_error("Replay files changed during preparation; waiting for another discovery message");
    } catch (const std::exception& error) {
      restartPending_ = false;
      std::cerr << "Replay restart: " << error.what() << '\n';
      return;
    }
    trace("REPLAY replacing messaging session");
    session_.reset(); // Stop/join first, then release Server and its dictionary.
    startSession(std::move(dictionary), *snapshotConfiguration, messages);
    replayVersion_ = version;
    restartPending_ = false;
    session_->link.start();
    trace("REPLAY session ready for " + sender);
    receive(sender, discovery); // Register even when this ALIVE was sent only once.
  }

  void startTimers() {
    using namespace std::chrono_literals;
    broadcastTimer_.expires_after(500ms);
    alertTimer_.expires_after(1s);
    windTimer_.expires_after(5s);
    repeat(broadcastTimer_, 500ms, [this] { session_->server.broadcast(currentTime()); });
    repeat(alertTimer_, 1s, [this] { session_->server.checkAlerts(currentTime()); });
    repeat(windTimer_, 5s, [this] {
      session_->server.publishWind();
      if (options_.verbose)
        reportStatus();
    });
  }

  // Re-arm each steady_timer after its handler. Preserve cadence, skipping
  // missed deadlines so a slow operation cannot trigger a burst of old ticks.
  void repeat(boost::asio::steady_timer& timer, std::chrono::steady_clock::duration period,
              std::function<void()> callback) {
    timer.async_wait([this, &timer, period, callback](const boost::system::error_code& error) {
      if (error == boost::asio::error::operation_aborted)
        return;
      if (error)
        throw boost::system::system_error(error);
      callback();
      auto next = timer.expiry() + period;
      const auto now = std::chrono::steady_clock::now();
      if (next <= now)
        next += ((now - next) / period + 1) * period;
      timer.expires_at(next);
      repeat(timer, period, callback);
    });
  }

  void reportStatus() {
    std::string peers;
    for (const auto& name : session_->link.peers())
      peers += (peers.empty() ? "" : ", ") + name;
    trace("STATUS rx=" + std::to_string(received_) + " tx=" + std::to_string(transmitted_) +
      " aircraft=" + std::to_string(session_->server.aircraft().size()) + " peers=[" + peers + "]");
    if (session_->server.aircraft().empty())
      trace("WAIT no registered aircraft; check ALIVE messages and registration errors above");
    for (const auto& [id, aircraft] : session_->server.aircraft())
      trace("AIRCRAFT " + id + " (" + aircraft.name + ") last_telemetry_seconds=" +
        std::to_string(currentTime() - aircraft.lastTelemetry));
  }
};
} // namespace

int runServer(Options options) {
  if (options.verbose) {
    std::cerr << "[server_cpp] START home=" << options.home.string()
              << " conf=" << (options.home / "conf/conf.xml").string() << '\n'
              << "[server_cpp] LOAD messages " << options.messages.string() << '\n'
              << "[server_cpp] PPRZLINK endpoint=" << options.bus << '\n';
  }
  Application application(std::move(options));
  application.run();
  return 0;
}
} // namespace server
