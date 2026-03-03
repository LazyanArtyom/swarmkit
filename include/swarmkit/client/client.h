#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace swarmkit::client {

struct ClientConfig {
    std::string address = "127.0.0.1:50061";
    std::string client_agent_id = "client";
    std::int64_t deadline_ms = 30000;
};

struct PingResult {
    bool ok = false;
    std::string agent_id;
    std::string version;
    std::int64_t unix_time_ms = 0;
    std::string error_message;
};

class Client {
   public:
    explicit Client(ClientConfig cfg);
    ~Client();

    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    PingResult Ping() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client