// SPDX-License-Identifier: GPL-2.0-or-later
#include "pprz_connection.h"
#include <boost/asio/post.hpp>
#include <pprzlink/IvyLink.h>
#include <atomic>
#include <iostream>
#include <system_error>
#include <thread>

namespace server {
namespace {
template<class T>
T valueOrThrow(std::expected<T, std::error_code> result) {
  if (!result)
    throw std::system_error(result.error());
  if constexpr (!std::is_void_v<T>)
    return std::move(*result);
}
}

struct PprzConnection::Impl {
  boost::asio::io_context& context;
  pprzlink::IvyLink link;
  std::atomic<bool> stopping = false;
  std::thread worker;

  Impl(const pprzlink::MessageDictionary& dictionary, const std::string& endpoint, boost::asio::io_context& context)
    : context(context), link(context, dictionary, "Paparazzi server", endpoint, false) {
    // PprzGCS discovers the server using this exact peer name.
    valueOrThrow(link.getBus().set_transport_error_callback(
      [](IvyClientPtr, std::error_code error, int systemError) {
        std::cerr << "PprzLink Ivy transport: " << error.message();
        if (systemError)
          std::cerr << " (" << std::error_code(systemError, std::generic_category()).message() << ")";
        std::cerr << '\n';
      }));
    link.onError([](const pprzlink::ReceiveError& error) {
      std::cerr << "PprzLink receive error: " << error.message << '\n';
      if (error.kind != pprzlink::ReceiveError::Kind::Decode)
        std::rethrow_exception(error.exception);
    });
  }
};

PprzConnection::PprzConnection(const pprzlink::MessageDictionary& dictionary,
  const std::string& endpoint, boost::asio::io_context& context)
  : impl_(std::make_unique<Impl>(dictionary, endpoint, context)) {}

PprzConnection::~PprzConnection() {
  stop();
}

void PprzConnection::bind(Receiver callback) {
  impl_->link.bind(pprzlink::ALL, [callback = std::move(callback)](const pprzlink::Message& message) {
    callback(std::get<std::string>(message.getSenderId()), message);
  });
}
void PprzConnection::bindRequest(const std::string& name, Answerer callback) {
  // The SDK owns the registration and cancels it when the link closes.
  impl_->link.bindRequest(name, std::move(callback));
}
void PprzConnection::send(const pprzlink::Message& message) {
  impl_->link.sendMessage(message);
}
std::vector<std::string> PprzConnection::peers() const {
  return valueOrThrow(impl_->link.getBus().applications());
}

void PprzConnection::start() {
  if (impl_->worker.joinable())
    throw std::logic_error("PprzLink connection already running");
  impl_->worker = std::thread([this] {
    try {
      impl_->link.run();
      if (!impl_->stopping)
        throw std::runtime_error("PprzLink transport stopped unexpectedly");
    } catch (...) {
      if (!impl_->stopping) {
        auto error = std::current_exception();
        boost::asio::post(impl_->context, [error] { std::rethrow_exception(error); });
      }
    }
  });
}
void PprzConnection::stop() noexcept {
  impl_->stopping = true;
  try {
    impl_->link.close(); // Suppress queued callbacks/replies even if Asio is stopped.
  } catch (...) {
    // stop() is noexcept; still request native shutdown and join below.
  }
  if (impl_->worker.joinable()) {
    (void)impl_->link.getBus().request_stop();
    impl_->worker.join();
  }
}
}
