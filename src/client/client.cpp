#include "swarmkit/client/client.h"

#include <grpcpp/grpcpp.h>

#include <chrono>
#include <utility>

#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

struct Client::Impl {
    ClientConfig cfg;
    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<swarmkit::v1::AgentService::Stub> stub;

    explicit Impl(ClientConfig c)
        : cfg(std::move(c)),
          channel(grpc::CreateChannel(cfg.address, grpc::InsecureChannelCredentials())),
          stub(swarmkit::v1::AgentService::NewStub(channel)) {}
};

Client::Client(ClientConfig cfg) : impl_(std::make_unique<Impl>(std::move(cfg))) {}
Client::~Client() = default;

PingResult Client::Ping() const {
    PingResult out;

    grpc::ClientContext ctx;
    if (impl_->cfg.deadline_ms > 0) {
        const auto deadline =
            std::chrono::system_clock::now() + std::chrono::milliseconds(impl_->cfg.deadline_ms);
        ctx.set_deadline(deadline);
    }

    swarmkit::v1::PingRequest req;
    req.set_agent_id(impl_->cfg.client_agent_id);

    swarmkit::v1::PingReply rep;
    const grpc::Status st = impl_->stub->Ping(&ctx, req, &rep);

    if (!st.ok()) {
        out.ok = false;
        out.error_message = st.error_message();
        return out;
    }

    out.ok = true;
    out.agent_id = rep.agent_id();
    out.version = rep.version();
    out.unix_time_ms = rep.unix_time_ms();
    return out;
}

}  // namespace swarmkit::client