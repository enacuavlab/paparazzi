// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <boost/asio/io_context.hpp>
#include <pprzlink/MessageDictionary.h>
#include <pprzlink/Message.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace server {
// The application's messaging boundary. Only PprzLink messages cross it;
// backend subscription objects, threads and request envelopes stay private.
class PprzConnection {
 public:
  using Receiver = std::function<void(const std::string&, const pprzlink::Message&)>;
  using Answerer = std::function<pprzlink::Message(const pprzlink::Message&)>;
  PprzConnection(const pprzlink::MessageDictionary& dictionary, const std::string& endpoint,
                 boost::asio::io_context& context);
  ~PprzConnection();
  PprzConnection(const PprzConnection&) = delete;
  PprzConnection& operator=(const PprzConnection&) = delete;
  void bind(Receiver callback);
  void bindRequest(const std::string& name, Answerer callback);
  void send(const pprzlink::Message& message);
  std::vector<std::string> peers() const;
  void start();
  // Suppress queued callbacks/replies and join the transport thread. Safe
  // after stopping the context; queued SDK callbacks remain inert on restart.
  void stop() noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
