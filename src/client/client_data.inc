// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

// Data-plane and artifact Client methods. Included from client.cpp so these
// methods can keep using the private PIMPL and proto conversion helpers without
// exposing them in the public SDK headers.

PublishMessageResult Client::PublishMessage(DataMessage message) const {
    PublishMessageResult out;
    const std::string correlation_id = MakeCorrelationId("data-message");
    if (message.source_id.empty()) {
        message.source_id = impl_->config.client_id;
    }
    if (message.message_id.empty()) {
        message.message_id = MakeCorrelationId("msg-" + message.source_id);
    }

    swarmkit::v1::PublishMessageRequest req;
    *req.mutable_message() = ToProtoDataMessage(message);
    swarmkit::v1::PublishMessageReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->PublishMessage(context, req, &rep);
                             });

    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.sequence = rep.sequence();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

PublishMessageResult Client::SendMessageToDrone(DataMessage message) const {
    PublishMessageResult out;
    const std::string correlation_id = MakeCorrelationId("data-route-message");
    if (message.target_id.empty()) {
        out.message = "target_id is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }
    if (message.source_id.empty()) {
        message.source_id = impl_->config.client_id;
    }
    if (message.message_id.empty()) {
        message.message_id = MakeCorrelationId("msg-" + message.source_id);
    }

    swarmkit::v1::PublishMessageRequest req;
    *req.mutable_message() = ToProtoDataMessage(message);
    swarmkit::v1::PublishMessageReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->SendMessageToDrone(context, req, &rep);
                             });

    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.sequence = rep.sequence();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kTransport);
    return out;
}

std::expected<Subscription, RpcError> Client::StartMessages(
    MessageSubscription subscription, DataMessageHandler on_message, TelemetryErrorHandler on_error,
    SubscriptionEventHandler on_event, SubscriptionOptions options) {
    if (subscription.subscriber_id.empty()) {
        subscription.subscriber_id = impl_->config.client_id;
    }
    if (!on_message) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kMessages, RpcStatusCode::kInvalidArgument,
            "message subscription callback must not be empty"));
    }

    auto generation = PrepareSubscriptionStart(impl_->messages, SubscriptionKind::kMessages,
                                               subscription.target_id, std::move(on_error),
                                               std::move(on_event), options);
    if (!generation.has_value()) {
        return std::unexpected(std::move(generation.error()));
    }

    try {
        const auto stream = impl_->messages;
        stream->worker =
            std::thread([this, stream, subscription = std::move(subscription),
                         on_message = std::move(on_message), on_error = stream->on_error]() mutable {
                RunMessageLoop(DataClientRuntime{.config = impl_->config,
                                                 .stub = *impl_->data_stub},
                               *stream, subscription, on_message, on_error);
            });
    } catch (const std::system_error& exc) {
        CancelAndJoinStream(*impl_->messages);
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kMessages, RpcStatusCode::kInternal,
            "failed to start message subscription worker: " + std::string(exc.what())));
    }

    auto state = std::make_shared<Subscription::State>();
    state->stream = impl_->messages;
    state->kind = SubscriptionKind::kMessages;
    state->generation = *generation;
    return Subscription(std::move(state));
}

void Client::StopMessages() {
    CancelAndJoinStream(*impl_->messages);
}

DataPeerListResult Client::ListDataPeers(bool refresh) const {
    DataPeerListResult out;
    const std::string correlation_id = MakeCorrelationId("data-peers");
    swarmkit::v1::DataPeerListRequest req;
    req.set_refresh(refresh);
    swarmkit::v1::DataPeerListReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->ListDataPeers(context, req, &rep);
                             });
    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    PopulateDataPeerListResult(&out, rep, correlation_id, attempt_count);
    return out;
}

DataPeerListResult Client::RefreshDataPeers(const std::vector<std::string>& drone_ids) const {
    DataPeerListResult out;
    const std::string correlation_id = MakeCorrelationId("data-peers-refresh");
    swarmkit::v1::DataPeerRefreshRequest req;
    for (const auto& drone_id : drone_ids) {
        req.add_drone_ids(drone_id);
    }
    swarmkit::v1::DataPeerListReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->RefreshDataPeers(context, req, &rep);
                             });
    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    PopulateDataPeerListResult(&out, rep, correlation_id, attempt_count);
    return out;
}

DataPeerListResult Client::UpsertDataPeer(const DataPeerConfig& peer) const {
    DataPeerListResult out;
    const std::string correlation_id = MakeCorrelationId("data-peer-upsert");
    swarmkit::v1::DataPeerConfig req = ToProtoDataPeerConfig(peer);
    swarmkit::v1::DataPeerListReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->UpsertDataPeer(context, req, &rep);
                             });
    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    PopulateDataPeerListResult(&out, rep, correlation_id, attempt_count);
    return out;
}

DataPeerListResult Client::RemoveDataPeer(const std::string& drone_id) const {
    DataPeerListResult out;
    const std::string correlation_id = MakeCorrelationId("data-peer-remove");
    swarmkit::v1::DataPeerRemoveRequest req;
    req.set_drone_id(drone_id);
    swarmkit::v1::DataPeerListReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->RemoveDataPeer(context, req, &rep);
                             });
    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    PopulateDataPeerListResult(&out, rep, correlation_id, attempt_count);
    return out;
}

namespace {

enum class ArtifactStreamRoute : std::uint8_t {
    kLocalUpload,
    kPeerRoute,
};

[[nodiscard]] ArtifactTransferResult StreamArtifactFile(const ClientConfig& config,
                                                        swarmkit::v1::DataService::Stub& data_stub,
                                                        const ArtifactUpload& upload,
                                                        ArtifactStreamRoute route) {
    ArtifactTransferResult out;
    const bool routed = route == ArtifactStreamRoute::kPeerRoute;
    const std::string correlation_id =
        MakeCorrelationId(routed ? "artifact-route" : "artifact-upload");
    out.correlation_id = correlation_id;

    if (routed && upload.descriptor.target_id.empty()) {
        out.message = "target_id is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    const std::filesystem::path file_path(upload.file_path);
    std::error_code fs_error;
    const auto file_size = std::filesystem::file_size(file_path, fs_error);
    if (upload.file_path.empty() || fs_error) {
        out.message = "artifact file does not exist or cannot be read";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    std::ifstream input(file_path, std::ios::binary);
    if (!input.is_open()) {
        out.message = "failed to open artifact file";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    ArtifactDescriptor descriptor = upload.descriptor;
    if (descriptor.filename.empty()) {
        descriptor.filename = file_path.filename().string();
    }
    if (descriptor.source_id.empty()) {
        descriptor.source_id = config.client_id;
    }
    if (descriptor.created_unix_ms == 0) {
        descriptor.created_unix_ms = NowUnixMs();
    }
    descriptor.size_bytes = static_cast<std::int64_t>(file_size);
    if (descriptor.sha256_hex.empty()) {
        descriptor.sha256_hex = core::internal::Sha256FileHex(file_path);
    }

    grpc::ClientContext context;
    ApplyUnaryClientContext(config, &context, correlation_id);
    swarmkit::v1::ArtifactDescriptor proto_reply;
    auto writer = routed ? data_stub.SendArtifactToDrone(&context, &proto_reply)
                         : data_stub.UploadArtifact(&context, &proto_reply);

    const std::size_t chunk_size = std::max<std::size_t>(
        1, upload.chunk_bytes == 0 ? kDefaultArtifactChunkBytes : upload.chunk_bytes);
    std::string buffer(chunk_size, '\0');
    std::int64_t offset = 0;
    int chunk_index = 0;
    bool wrote_all = true;
    if (descriptor.size_bytes == 0) {
        swarmkit::v1::ArtifactChunk chunk;
        chunk.set_transfer_id(correlation_id);
        *chunk.mutable_artifact() = ToProtoArtifactDescriptor(descriptor);
        chunk.set_offset(0);
        chunk.set_chunk_index(0);
        chunk.set_final_chunk(true);
        wrote_all = writer->Write(chunk);
    }
    while (input.good()) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize read = input.gcount();
        if (read <= 0) {
            break;
        }
        swarmkit::v1::ArtifactChunk chunk;
        chunk.set_transfer_id(correlation_id);
        if (chunk_index == 0) {
            *chunk.mutable_artifact() = ToProtoArtifactDescriptor(descriptor);
        }
        chunk.set_offset(offset);
        chunk.set_chunk_index(chunk_index);
        chunk.set_data(buffer.data(), static_cast<std::size_t>(read));
        offset += read;
        chunk.set_final_chunk(offset >= descriptor.size_bytes);
        if (!writer->Write(chunk)) {
            wrote_all = false;
            break;
        }
        ++chunk_index;
    }
    writer->WritesDone();
    const grpc::Status status = writer->Finish();

    if (!wrote_all && status.ok()) {
        out.message = routed ? "artifact route stream closed before all chunks were accepted"
                             : "artifact upload stream closed before all chunks were accepted";
        PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                           RpcStatusCode::kUnavailable, out.message, out.message, correlation_id,
                           1);
        return out;
    }
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, 1);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = true;
    out.message = routed ? "artifact delivered" : "artifact uploaded";
    out.descriptor = ToArtifactDescriptor(proto_reply);
    PopulateSuccessError(&out.error, correlation_id, 1);
    return out;
}

}  // namespace

ArtifactTransferResult Client::UploadArtifact(const ArtifactUpload& upload) const {
    return StreamArtifactFile(impl_->config, *impl_->data_stub, upload,
                              ArtifactStreamRoute::kLocalUpload);
}

ArtifactTransferResult Client::SendArtifactToDrone(const ArtifactUpload& upload) const {
    return StreamArtifactFile(impl_->config, *impl_->data_stub, upload,
                              ArtifactStreamRoute::kPeerRoute);
}

ArtifactTransferStatusResult Client::StartArtifactTransfer(const ArtifactUpload& upload,
                                                           bool route_to_target) const {
    ArtifactTransferStatusResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-transfer-start");
    out.correlation_id = correlation_id;

    if (upload.file_path.empty()) {
        out.message = "artifact source path is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }
    if (route_to_target && upload.descriptor.target_id.empty()) {
        out.message = "target_id is required for routed artifact transfer";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    ArtifactDescriptor descriptor = upload.descriptor;
    if (descriptor.source_id.empty()) {
        descriptor.source_id = impl_->config.client_id;
    }
    if (descriptor.created_unix_ms == 0) {
        descriptor.created_unix_ms = NowUnixMs();
    }

    swarmkit::v1::ArtifactTransferStartRequest req;
    req.set_source_path(upload.file_path);
    *req.mutable_artifact() = ToProtoArtifactDescriptor(descriptor);
    req.set_route_to_target(route_to_target);
    req.set_chunk_bytes(static_cast<int>(upload.chunk_bytes));
    swarmkit::v1::ArtifactTransferReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->StartArtifactTransfer(context, req, &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.transfer = ToArtifactTransferStatus(rep.transfer());
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

ArtifactTransferStatusResult Client::GetArtifactTransfer(const std::string& transfer_id) const {
    ArtifactTransferStatusResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-transfer-get");
    out.correlation_id = correlation_id;
    if (transfer_id.empty()) {
        out.message = "transfer_id is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }
    swarmkit::v1::ArtifactTransferRequest req;
    req.set_transfer_id(transfer_id);
    swarmkit::v1::ArtifactTransferReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->GetArtifactTransfer(context, req, &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.transfer = ToArtifactTransferStatus(rep.transfer());
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

ArtifactTransferStatusResult Client::CancelArtifactTransfer(const std::string& transfer_id) const {
    ArtifactTransferStatusResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-transfer-cancel");
    out.correlation_id = correlation_id;
    if (transfer_id.empty()) {
        out.message = "transfer_id is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }
    swarmkit::v1::ArtifactTransferRequest req;
    req.set_transfer_id(transfer_id);
    swarmkit::v1::ArtifactTransferReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->CancelArtifactTransfer(context, req,
                                                                                 &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.transfer = ToArtifactTransferStatus(rep.transfer());
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

ArtifactListResult Client::ListArtifacts(std::string source_id, std::string target_id,
                                         bool include_expired) const {
    return ListArtifacts(ArtifactListOptions{
        .source_id = std::move(source_id),
        .target_id = std::move(target_id),
        .include_expired = include_expired,
    });
}

ArtifactListResult Client::ListArtifacts(const ArtifactListOptions& options) const {
    ArtifactListResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-list");
    out.correlation_id = correlation_id;

    swarmkit::v1::ArtifactListRequest req;
    req.set_source_id(options.source_id);
    req.set_target_id(options.target_id);
    req.set_include_expired(options.include_expired);
    req.set_page_size(options.page_size);
    req.set_page_token(options.page_token);
    swarmkit::v1::ArtifactListReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->ListArtifacts(context, req, &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = true;
    out.message = "artifacts listed";
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.next_page_token = rep.next_page_token();
    out.total_count = rep.total_count();
    out.artifacts.reserve(static_cast<std::size_t>(rep.artifacts_size()));
    for (const auto& artifact : rep.artifacts()) {
        out.artifacts.push_back(ToArtifactDescriptor(artifact));
    }
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

ArtifactTransferResult Client::GetArtifact(const std::string& artifact_id) const {
    ArtifactTransferResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-info");
    out.correlation_id = correlation_id;
    if (artifact_id.empty()) {
        out.message = "artifact_id is required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    swarmkit::v1::ArtifactRequest req;
    req.set_artifact_id(artifact_id);
    swarmkit::v1::ArtifactReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->GetArtifact(context, req, &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.descriptor = ToArtifactDescriptor(rep.artifact());
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

namespace {

[[nodiscard]] ArtifactTransferResult DownloadArtifactToFile(const ClientConfig& config,
                                                            swarmkit::v1::DataService::Stub& data_stub,
                                                            std::string_view artifact_id,
                                                            const std::filesystem::path& final_path,
                                                            bool resume) {
    ArtifactTransferResult out;
    const std::string correlation_id =
        MakeCorrelationId(resume ? "artifact-download-resume" : "artifact-download");
    out.correlation_id = correlation_id;

    if (artifact_id.empty() || final_path.empty()) {
        out.message = "artifact_id and output_path are required";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    grpc::ClientContext context;
    ApplyUnaryClientContext(config, &context, correlation_id);
    swarmkit::v1::ArtifactRequest req;
    req.set_artifact_id(std::string(artifact_id));

    const std::filesystem::path tmp_path =
        final_path.parent_path() / (final_path.filename().string() + ".part");
    std::error_code fs_error;
    if (!final_path.parent_path().empty()) {
        std::filesystem::create_directories(final_path.parent_path(), fs_error);
        if (fs_error) {
            out.message = "failed to create artifact output directory path=" +
                          final_path.parent_path().string() + " error=" + fs_error.message();
            PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                               RpcStatusCode::kFailedPrecondition, out.message, out.message,
                               correlation_id, 0);
            return out;
        }
    }
    std::int64_t start_offset = 0;
    if (resume && std::filesystem::exists(tmp_path, fs_error)) {
        const auto partial_size = std::filesystem::file_size(tmp_path, fs_error);
        if (!fs_error) {
            start_offset = static_cast<std::int64_t>(partial_size);
        }
    }
    req.set_offset_bytes(start_offset);
    auto reader = data_stub.DownloadArtifact(&context, req);

    const auto output_mode = std::ios::binary | (resume ? std::ios::app : std::ios::trunc);
    std::ofstream output(tmp_path, output_mode);
    if (!output.is_open()) {
        out.message = "failed to open artifact output file path=" + tmp_path.string();
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kInvalidArgument, out.message, out.message,
                           correlation_id, 0);
        return out;
    }

    swarmkit::v1::ArtifactChunk chunk;
    bool saw_descriptor = false;
    bool saw_final = false;
    std::int64_t received_bytes = start_offset;
    int expected_chunk_index = 0;
    while (reader->Read(&chunk)) {
        if (saw_final) {
            output.close();
            std::filesystem::remove(tmp_path, fs_error);
            out.message = "artifact download received data after final chunk";
            PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                               RpcStatusCode::kInternal, out.message, out.message, correlation_id,
                               1);
            return out;
        }
        if (!saw_descriptor && chunk.has_artifact()) {
            out.descriptor = ToArtifactDescriptor(chunk.artifact());
            saw_descriptor = true;
        }
        if (chunk.chunk_index() != expected_chunk_index) {
            output.close();
            std::filesystem::remove(tmp_path, fs_error);
            out.message = "artifact download chunk index mismatch";
            PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                               RpcStatusCode::kInternal, out.message, out.message, correlation_id,
                               1);
            return out;
        }
        if (chunk.offset() != received_bytes) {
            output.close();
            std::filesystem::remove(tmp_path, fs_error);
            out.message = "artifact download chunk offset mismatch";
            PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                               RpcStatusCode::kInternal, out.message, out.message, correlation_id,
                               1);
            return out;
        }
        if (!WriteBytes(output,
                        std::span<const char>{chunk.data().data(), chunk.data().size()})) {
            output.close();
            std::filesystem::remove(tmp_path, fs_error);
            out.message = "failed to write artifact output file path=" + tmp_path.string();
            PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                               RpcStatusCode::kFailedPrecondition, out.message, out.message,
                               correlation_id, 1);
            return out;
        }
        received_bytes += static_cast<std::int64_t>(chunk.data().size());
        ++expected_chunk_index;
        saw_final = saw_final || chunk.final_chunk();
    }
    output.flush();
    output.close();

    const grpc::Status status = reader->Finish();
    if (!status.ok()) {
        std::filesystem::remove(tmp_path, fs_error);
        PopulateTransportError(&out.error, status, correlation_id, 1);
        out.message = out.error.user_message;
        return out;
    }
    if (!saw_descriptor || !saw_final) {
        std::filesystem::remove(tmp_path, fs_error);
        out.message = "artifact download ended before descriptor/final chunk";
        PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                           RpcStatusCode::kInternal, out.message, out.message, correlation_id, 1);
        return out;
    }
    if (!out.descriptor.sha256_hex.empty()) {
        const std::string downloaded_hash = core::internal::Sha256FileHex(tmp_path);
        if (downloaded_hash != out.descriptor.sha256_hex) {
            std::filesystem::remove(tmp_path, fs_error);
            out.message = "artifact download sha256 mismatch";
            PopulateTypedError(&out.error, core::ErrorDomain::kTransport,
                               RpcStatusCode::kInternal, out.message, out.message, correlation_id,
                               1);
            return out;
        }
    }

    std::filesystem::rename(tmp_path, final_path, fs_error);
    if (fs_error) {
        std::filesystem::remove(final_path, fs_error);
        fs_error.clear();
        std::filesystem::rename(tmp_path, final_path, fs_error);
    }
    if (fs_error) {
        std::filesystem::remove(tmp_path, fs_error);
        out.message = "failed to finalize artifact output file";
        PopulateTypedError(&out.error, core::ErrorDomain::kValidation,
                           RpcStatusCode::kFailedPrecondition, out.message, out.message,
                           correlation_id, 1);
        return out;
    }

    out.ok = true;
    out.message = resume && start_offset > 0 ? "artifact download resumed"
                                             : "artifact downloaded";
    PopulateSuccessError(&out.error, correlation_id, 1);
    return out;
}

}  // namespace

ArtifactTransferResult Client::DownloadArtifact(const std::string& artifact_id,
                                                const std::string& output_path) const {
    return DownloadArtifactToFile(impl_->config, *impl_->data_stub, artifact_id,
                                  std::filesystem::path(output_path), false);
}

ArtifactTransferResult Client::ResumeArtifactDownload(const std::string& artifact_id,
                                                      const std::string& output_path) const {
    return DownloadArtifactToFile(impl_->config, *impl_->data_stub, artifact_id,
                                  std::filesystem::path(output_path), true);
}

ArtifactTransferResult Client::AnnounceArtifact(ArtifactDescriptor descriptor) const {
    ArtifactTransferResult out;
    const std::string correlation_id = MakeCorrelationId("artifact-announce");

    swarmkit::v1::ArtifactReply rep;
    swarmkit::v1::ArtifactDescriptor req = ToProtoArtifactDescriptor(descriptor);
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->data_stub->AnnounceArtifact(context, req, &rep);
                             });

    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    out.descriptor = ToArtifactDescriptor(rep.artifact());
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kInternal);
    return out;
}

CommandResult Client::LockAuthority(const std::string& drone_id, std::int64_t ttl_ms) const {
    CommandResult out;
    const std::string kCorrelationId = MakeCorrelationId("lock");

    swarmkit::v1::LockAuthorityRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    req.set_ttl_ms(ttl_ms);

    swarmkit::v1::LockAuthorityReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->LockAuthority(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kAuthority);
    return out;
}
