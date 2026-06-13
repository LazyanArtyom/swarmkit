// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

constexpr std::int64_t kDefaultArtifactUploadSessionTtlMs = 60LL * 60LL * 1000LL;

class ArtifactStore {
   public:
    struct StoredArtifact {
        swarmkit::v1::ArtifactDescriptor descriptor;
        std::filesystem::path storage_path;
    };

    struct ListPage {
        std::vector<swarmkit::v1::ArtifactDescriptor> artifacts;
        int total_count{0};
        std::string next_page_token;
    };

    explicit ArtifactStore(AgentConfig config) : config_(std::move(config)) {
        LoadIndex();
    }

    [[nodiscard]] std::filesystem::path Root() const {
        return ArtifactStorageRoot(config_);
    }

    [[nodiscard]] std::filesystem::path TempDir() const {
        return Root() / "tmp";
    }

    [[nodiscard]] std::optional<StoredArtifact> Find(std::string_view artifact_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = artifacts_.find(std::string(artifact_id));
        if (iter == artifacts_.end()) {
            return std::nullopt;
        }
        if (ArtifactExpired(iter->second.descriptor, NowUnixMs())) {
            artifacts_.erase(iter);
            return std::nullopt;
        }
        return iter->second;
    }

    [[nodiscard]] std::optional<StoredArtifact> FindIncludingExpired(std::string_view artifact_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = artifacts_.find(std::string(artifact_id));
        if (iter == artifacts_.end()) {
            return std::nullopt;
        }
        return iter->second;
    }

    void Put(swarmkit::v1::ArtifactDescriptor descriptor, std::filesystem::path storage_path) {
        const std::string artifact_id = descriptor.artifact_id();
        std::lock_guard<std::mutex> lock(mutex_);
        artifacts_[artifact_id] = StoredArtifact{.descriptor = std::move(descriptor),
                                                 .storage_path = std::move(storage_path)};
    }

    void Remove(std::string_view artifact_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        artifacts_.erase(std::string(artifact_id));
    }

    [[nodiscard]] ListPage List(std::string_view source_id, std::string_view target_id,
                                bool include_expired, std::size_t start_index,
                                std::size_t page_size) {
        const std::int64_t now_ms = NowUnixMs();
        std::vector<std::string> expired_ids;
        std::vector<swarmkit::v1::ArtifactDescriptor> matching;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& [artifact_id, stored] : artifacts_) {
                const auto& descriptor = stored.descriptor;
                const bool expired = ArtifactExpired(descriptor, now_ms);
                if (expired && !include_expired) {
                    expired_ids.push_back(artifact_id);
                    continue;
                }
                if (!source_id.empty() && descriptor.source_id() != source_id) {
                    continue;
                }
                if (!target_id.empty() && descriptor.target_id() != target_id) {
                    continue;
                }
                matching.push_back(descriptor);
            }
            for (const std::string& artifact_id : expired_ids) {
                artifacts_.erase(artifact_id);
            }
        }

        std::ranges::sort(matching, [](const auto& lhs, const auto& rhs) {
            if (lhs.created_unix_ms() != rhs.created_unix_ms()) {
                return lhs.created_unix_ms() < rhs.created_unix_ms();
            }
            return lhs.artifact_id() < rhs.artifact_id();
        });

        ListPage page;
        page.total_count = static_cast<int>(matching.size());
        if (start_index < matching.size()) {
            const std::size_t end_index = std::min(matching.size(), start_index + page_size);
            const auto start_offset =
                static_cast<std::vector<swarmkit::v1::ArtifactDescriptor>::difference_type>(
                    start_index);
            const auto end_offset =
                static_cast<std::vector<swarmkit::v1::ArtifactDescriptor>::difference_type>(
                    end_index);
            page.artifacts.insert(page.artifacts.end(), matching.begin() + start_offset,
                                  matching.begin() + end_offset);
            if (end_index < matching.size()) {
                page.next_page_token = std::to_string(end_index);
            }
        }
        return page;
    }

    [[nodiscard]] static core::Result PersistMetadata(
        const swarmkit::v1::ArtifactDescriptor& descriptor,
        const std::filesystem::path& storage_path) {
        const std::filesystem::path metadata_path = MetadataPath(storage_path);
        std::error_code fs_error;
        std::filesystem::create_directories(metadata_path.parent_path(), fs_error);
        if (fs_error) {
            return core::Result::Failed("failed to create artifact metadata dir path=" +
                                        metadata_path.parent_path().string() +
                                        " error=" + fs_error.message());
        }
        std::ofstream output(metadata_path, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) {
            return core::Result::Failed("failed to open artifact metadata path=" +
                                        metadata_path.string());
        }
        if (!descriptor.SerializeToOstream(&output)) {
            return core::Result::Failed("failed to serialize artifact metadata path=" +
                                        metadata_path.string());
        }
        output.flush();
        if (!output.good()) {
            return core::Result::Failed("failed to write artifact metadata path=" +
                                        metadata_path.string());
        }
        return core::Result::Success();
    }

   private:
    [[nodiscard]] static std::filesystem::path MetadataPath(
        const std::filesystem::path& storage_path) {
        return storage_path.parent_path() / std::string(kArtifactMetadataFilename);
    }

    void LoadIndex() {
        const std::filesystem::path root = Root();
        std::error_code fs_error;
        if (!std::filesystem::exists(root, fs_error)) {
            return;
        }
        int loaded = 0;
        for (std::filesystem::recursive_directory_iterator iter(
                 root, std::filesystem::directory_options::skip_permission_denied, fs_error);
             !fs_error && iter != std::filesystem::recursive_directory_iterator{};
             iter.increment(fs_error)) {
            if (!iter->is_regular_file(fs_error) ||
                iter->path().filename() != kArtifactMetadataFilename) {
                continue;
            }
            swarmkit::v1::ArtifactDescriptor descriptor;
            std::ifstream input(iter->path(), std::ios::binary);
            if (!input.is_open() || !descriptor.ParseFromIstream(&input) ||
                descriptor.artifact_id().empty() || descriptor.filename().empty()) {
                core::Logger::WarnFmt("artifact_index_load path={} result=skipped",
                                      iter->path().string());
                continue;
            }
            const std::filesystem::path storage_path =
                iter->path().parent_path() / SanitizedPathComponent(descriptor.filename());
            if (!std::filesystem::exists(storage_path, fs_error)) {
                core::Logger::WarnFmt(
                    "artifact_index_load artifact_id={} path={} result=missing_file",
                    descriptor.artifact_id(), storage_path.string());
                continue;
            }
            artifacts_[descriptor.artifact_id()] =
                StoredArtifact{.descriptor = descriptor, .storage_path = storage_path};
            ++loaded;
        }
        if (fs_error) {
            core::Logger::WarnFmt("artifact_index_load root={} result=partial error={}",
                                  root.string(), fs_error.message());
        }
        core::Logger::InfoFmt("artifact_index_load root={} loaded={}", root.string(), loaded);
    }

    AgentConfig config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, StoredArtifact> artifacts_;
};

class PeerRegistry {
   public:
    explicit PeerRegistry(AgentConfig config)
        : config_(std::move(config)), peers_(config_.data.peers) {}

    [[nodiscard]] static std::expected<DataPeerConfig, core::Result> ToPeerConfig(
        const swarmkit::v1::DataPeerConfig& proto) {
        DataPeerConfig peer;
        peer.drone_id = proto.drone_id();
        peer.address = proto.address();
        if (!proto.transport_security().empty()) {
            const auto parsed = core::ParseTransportSecurityMode(proto.transport_security());
            if (!parsed.has_value()) {
                return std::unexpected(core::Result::Rejected(parsed.error()));
            }
            peer.transport_security = *parsed;
        }
        peer.root_ca_cert_path = proto.root_ca_cert_path();
        peer.cert_chain_path = proto.cert_chain_path();
        peer.private_key_path = proto.private_key_path();
        peer.server_authority_override = proto.server_authority_override();
        return peer;
    }

    [[nodiscard]] std::optional<DataPeerConfig> Find(std::string_view drone_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = std::ranges::find_if(
            peers_, [drone_id](const DataPeerConfig& peer) { return peer.drone_id == drone_id; });
        if (iter == peers_.end()) {
            return std::nullopt;
        }
        return *iter;
    }

    void Upsert(const DataPeerConfig& peer) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = std::ranges::find_if(peers_, [&](const DataPeerConfig& existing) {
            return existing.drone_id == peer.drone_id;
        });
        if (iter == peers_.end()) {
            peers_.push_back(peer);
        } else {
            *iter = peer;
        }
        statuses_[peer.drone_id] = DefaultStatus(peer);
    }

    [[nodiscard]] bool Remove(std::string_view drone_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto old_size = peers_.size();
        std::erase_if(peers_,
                      [&](const DataPeerConfig& peer) { return peer.drone_id == drone_id; });
        statuses_.erase(std::string(drone_id));
        return peers_.size() != old_size;
    }

    void Refresh(const std::unordered_set<std::string>& requested) {
        std::vector<DataPeerConfig> peers;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            peers = peers_;
        }
        for (const auto& peer : peers) {
            if (!requested.empty() && !requested.contains(peer.drone_id)) {
                continue;
            }
            swarmkit::v1::DataPeerStatus status = RefreshOne(peer);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                statuses_[peer.drone_id] = status;
            }
            core::Logger::InfoFmt(
                "data_peer_refresh agent={} peer={} address={} state={} rtt_ms={} message={}",
                config_.agent_id, status.drone_id(), status.address(),
                static_cast<int>(status.state()), status.round_trip_ms(), status.message());
        }
    }

    void FillReply(std::string_view correlation_id, bool success, std::string_view message,
                   swarmkit::v1::ErrorCode error_code,
                   swarmkit::v1::DataPeerListReply* reply) const {
        if (reply == nullptr) {
            return;
        }
        reply->set_correlation_id(std::string(correlation_id));
        reply->set_ok(success);
        reply->set_message(std::string(message));
        reply->set_error_code(error_code);
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& peer : peers_) {
            const auto iter = statuses_.find(peer.drone_id);
            if (iter != statuses_.end()) {
                *reply->add_peers() = iter->second;
            } else {
                *reply->add_peers() = DefaultStatus(peer);
            }
        }
    }

    void MarkRouteResult(const DataPeerConfig& peer, bool route_success, std::int64_t latency_ms,
                         std::string_view message) {
        swarmkit::v1::DataPeerStatus status = DefaultStatus(peer);
        const std::int64_t now_ms = NowUnixMs();
        status.set_last_checked_unix_ms(now_ms);
        status.set_round_trip_ms(std::max<std::int64_t>(0, latency_ms));
        status.set_message(std::string(message));
        if (route_success) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_READY);
            status.set_last_success_unix_ms(now_ms);
        } else {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_UNREACHABLE);
            status.set_last_failure_unix_ms(now_ms);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        statuses_[peer.drone_id] = std::move(status);
    }

   private:
    [[nodiscard]] swarmkit::v1::DataPeerStatus DefaultStatus(const DataPeerConfig& peer) const {
        swarmkit::v1::DataPeerStatus status;
        status.set_drone_id(peer.drone_id);
        status.set_address(peer.address);
        status.set_transport_security(
            TransportSecurityName(EffectivePeerTransportSecurity(peer, config_.security)));
        status.set_state(swarmkit::v1::DATA_PEER_STATE_UNKNOWN);
        status.set_message("not checked");
        return status;
    }

    [[nodiscard]] swarmkit::v1::DataPeerStatus RefreshOne(const DataPeerConfig& peer) const {
        swarmkit::v1::DataPeerStatus status = DefaultStatus(peer);
        const std::int64_t started_ms = NowUnixMs();
        status.set_last_checked_unix_ms(started_ms);
        if (const core::Result validation = peer.Validate(); !validation.IsOk()) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_MISCONFIGURED);
            status.set_last_failure_unix_ms(started_ms);
            status.set_message(validation.message);
            return status;
        }

        auto channel = MakePeerChannel(peer, config_.security);
        auto stub = swarmkit::v1::AgentService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{3});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), MakeCorrelationId("peer-check"));
        swarmkit::v1::PingRequest request;
        request.set_agent_id(config_.agent_id);
        swarmkit::v1::PingReply reply;
        const grpc::Status ping_status = stub->Ping(&peer_ctx, request, &reply);
        const std::int64_t finished_ms = NowUnixMs();
        status.set_round_trip_ms(std::max<std::int64_t>(0, finished_ms - started_ms));
        if (!ping_status.ok()) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_UNREACHABLE);
            status.set_last_failure_unix_ms(finished_ms);
            status.set_message(ping_status.error_message().empty() ? "peer ping failed"
                                                                   : ping_status.error_message());
            return status;
        }
        status.set_state(swarmkit::v1::DATA_PEER_STATE_READY);
        status.set_last_success_unix_ms(finished_ms);
        status.set_message("ready agent_id=" + reply.agent_id());
        return status;
    }

    AgentConfig config_;
    mutable std::mutex mutex_;
    std::vector<DataPeerConfig> peers_;
    std::unordered_map<std::string, swarmkit::v1::DataPeerStatus> statuses_;
};

class DataRouter {
   public:
    DataRouter(AgentConfig config, PeerRegistry* peer_registry)
        : config_(std::move(config)), peer_registry_(peer_registry) {}

    [[nodiscard]] std::optional<DataPeerConfig> FindPeer(std::string_view drone_id) const {
        return peer_registry_ == nullptr ? std::nullopt : peer_registry_->Find(drone_id);
    }

    [[nodiscard]] std::shared_ptr<grpc::Channel> MakeChannel(const DataPeerConfig& peer) const {
        return MakePeerChannel(peer, config_.security);
    }

    void MarkRouteResult(const DataPeerConfig& peer, bool route_success, std::int64_t latency_ms,
                         std::string_view message) const {
        if (peer_registry_ != nullptr) {
            peer_registry_->MarkRouteResult(peer, route_success, latency_ms, message);
        }
    }

   private:
    AgentConfig config_;
    PeerRegistry* peer_registry_{nullptr};
};

struct ArtifactTransferOutcome {
    bool ok{false};
    swarmkit::v1::ErrorCode error_code{swarmkit::v1::ERROR_CODE_INTERNAL};
    std::string message;
    swarmkit::v1::ArtifactDescriptor descriptor;
};

class ArtifactTransferManager {
   public:
    using Worker = std::function<void(
        std::string, const swarmkit::v1::ArtifactTransferStartRequest&, const std::stop_token&)>;

    struct StartResult {
        bool accepted{false};
        swarmkit::v1::ArtifactTransferStatus status;
    };

    explicit ArtifactTransferManager(const DataPlaneConfig& config)
        : max_concurrent_(std::max(1, config.max_concurrent_artifact_transfers)),
          max_queued_(std::max(0, config.max_queued_artifact_transfers)),
          max_per_peer_(std::max(1, config.max_concurrent_artifact_transfers_per_peer)),
          peer_bytes_per_second_(std::max(0, config.artifact_peer_bytes_per_second)) {
        workers_.reserve(static_cast<std::size_t>(max_concurrent_));
        for (int index = 0; index < max_concurrent_; ++index) {
            workers_.emplace_back(
                [this](const std::stop_token& stop_token) { WorkerLoop(stop_token); });
        }
    }

    ~ArtifactTransferManager() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutting_down_ = true;
            for (auto& [unused, runtime] : transfers_) {
                static_cast<void>(unused);
                runtime.stop_source.request_stop();
            }
        }
        cv_.notify_all();
    }

    ArtifactTransferManager(const ArtifactTransferManager&) = delete;
    ArtifactTransferManager& operator=(const ArtifactTransferManager&) = delete;

    [[nodiscard]] StartResult Start(const swarmkit::v1::ArtifactTransferStartRequest& request,
                                    std::int64_t file_size, Worker worker) {
        const std::string transfer_id = MakeCorrelationId("artifact-transfer");
        swarmkit::v1::ArtifactTransferStatus status;
        status.set_transfer_id(transfer_id);
        status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_QUEUED);
        *status.mutable_artifact() = request.artifact();
        status.set_bytes_total(file_size);
        status.set_started_unix_ms(NowUnixMs());
        status.set_updated_unix_ms(status.started_unix_ms());
        status.set_message("queued");
        status.set_error_code(swarmkit::v1::ERROR_CODE_NONE);

        const std::string peer_key = PeerKey(request);
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutting_down_) {
            status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED);
            status.set_message("artifact transfer manager is shutting down");
            status.set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
            status.set_completed_unix_ms(NowUnixMs());
            return StartResult{.accepted = false, .status = std::move(status)};
        }
        if (RunningCountLocked() >= static_cast<std::size_t>(max_concurrent_) &&
            queue_.size() >= static_cast<std::size_t>(max_queued_)) {
            status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED);
            status.set_message("artifact transfer queue is full");
            status.set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            status.set_completed_unix_ms(NowUnixMs());
            return StartResult{.accepted = false, .status = std::move(status)};
        }
        if (!peer_key.empty() &&
            PeerActiveCountLocked(peer_key) >= static_cast<std::size_t>(max_per_peer_)) {
            status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED);
            status.set_message("artifact transfer peer concurrency limit reached for " + peer_key);
            status.set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            status.set_completed_unix_ms(NowUnixMs());
            return StartResult{.accepted = false, .status = std::move(status)};
        }

        Runtime runtime;
        runtime.status = status;
        runtime.request = request;
        runtime.worker = std::move(worker);
        runtime.peer_key = peer_key;
        transfers_.emplace(transfer_id, std::move(runtime));
        queue_.push_back(transfer_id);
        cv_.notify_one();
        return StartResult{.accepted = true, .status = std::move(status)};
    }

    [[nodiscard]] std::optional<swarmkit::v1::ArtifactTransferStatus> Find(
        std::string_view transfer_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = transfers_.find(std::string(transfer_id));
        if (iter == transfers_.end()) {
            return std::nullopt;
        }
        return iter->second.status;
    }

    [[nodiscard]] std::optional<swarmkit::v1::ArtifactTransferStatus> Cancel(
        std::string_view transfer_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = transfers_.find(std::string(transfer_id));
        if (iter == transfers_.end()) {
            return std::nullopt;
        }
        iter->second.stop_source.request_stop();
        if (iter->second.status.state() != swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED &&
            iter->second.status.state() != swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED &&
            iter->second.status.state() != swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED) {
            iter->second.status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED);
            iter->second.status.set_message("cancel requested");
            iter->second.status.set_updated_unix_ms(NowUnixMs());
        }
        cv_.notify_all();
        return iter->second.status;
    }

    void Update(std::string_view transfer_id,
                const std::function<void(swarmkit::v1::ArtifactTransferStatus*)>& update) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = transfers_.find(std::string(transfer_id));
        if (iter == transfers_.end()) {
            return;
        }
        update(&iter->second.status);
        iter->second.status.set_updated_unix_ms(NowUnixMs());
    }

    void Complete(std::string_view transfer_id, swarmkit::v1::ArtifactTransferState state,
                  swarmkit::v1::ErrorCode error_code, std::string_view message,
                  const swarmkit::v1::ArtifactDescriptor& descriptor) {
        Update(transfer_id, [&](swarmkit::v1::ArtifactTransferStatus* status) {
            status->set_state(state);
            status->set_error_code(error_code);
            status->set_message(std::string(message));
            status->set_completed_unix_ms(NowUnixMs());
            if (!descriptor.artifact_id().empty()) {
                *status->mutable_artifact() = descriptor;
            }
        });
    }

    [[nodiscard]] bool ThrottlePeerBandwidth(std::string_view peer_key, std::size_t bytes,
                                             const std::stop_token& stop_token) {
        if (peer_key.empty() || bytes == 0 || peer_bytes_per_second_ <= 0) {
            return !stop_token.stop_requested();
        }

        const auto reservation = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(static_cast<double>(bytes) /
                                          static_cast<double>(peer_bytes_per_second_)));
        std::chrono::steady_clock::time_point release_at;
        {
            std::lock_guard<std::mutex> lock(bandwidth_mutex_);
            const std::string key(peer_key);
            const auto now = std::chrono::steady_clock::now();
            auto& next_available = peer_next_send_time_[key];
            if (next_available < now) {
                next_available = now;
            }
            next_available += reservation;
            release_at = next_available;
        }

        while (!stop_token.stop_requested()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= release_at) {
                return true;
            }
            const auto remaining = release_at - now;
            std::this_thread::sleep_for(std::min<std::chrono::steady_clock::duration>(
                remaining, std::chrono::milliseconds{25}));
        }
        return false;
    }

   private:
    [[nodiscard]] static bool Terminal(const swarmkit::v1::ArtifactTransferStatus& status) {
        return status.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED ||
               status.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED ||
               status.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED;
    }

    [[nodiscard]] static std::string PeerKey(
        const swarmkit::v1::ArtifactTransferStartRequest& request) {
        if (!request.route_to_target()) {
            return {};
        }
        return request.artifact().target_id();
    }

    [[nodiscard]] std::size_t RunningCountLocked() const {
        std::size_t count = 0;
        for (const auto& [unused, runtime] : transfers_) {
            static_cast<void>(unused);
            if (runtime.status.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_RUNNING) {
                ++count;
            }
        }
        return count;
    }

    [[nodiscard]] std::size_t PeerActiveCountLocked(std::string_view peer_key) const {
        std::size_t count = 0;
        for (const auto& [unused, runtime] : transfers_) {
            static_cast<void>(unused);
            if (runtime.peer_key == peer_key && !Terminal(runtime.status)) {
                ++count;
            }
        }
        return count;
    }

    void WorkerLoop(const std::stop_token& worker_stop) {
        while (!worker_stop.stop_requested()) {
            std::string transfer_id;
            swarmkit::v1::ArtifactTransferStartRequest request;
            Worker worker;
            std::stop_token task_stop;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds{100}, [&] {
                    return shutting_down_ || !queue_.empty() || worker_stop.stop_requested();
                });
                if ((shutting_down_ || worker_stop.stop_requested()) && queue_.empty()) {
                    return;
                }
                if (queue_.empty()) {
                    continue;
                }
                transfer_id = std::move(queue_.front());
                queue_.pop_front();
                auto iter = transfers_.find(transfer_id);
                if (iter == transfers_.end() || Terminal(iter->second.status)) {
                    cv_.notify_all();
                    continue;
                }
                iter->second.status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_RUNNING);
                iter->second.status.set_message("running");
                iter->second.status.set_updated_unix_ms(NowUnixMs());
                request = iter->second.request;
                worker = iter->second.worker;
                task_stop = iter->second.stop_source.get_token();
            }

            try {
                worker(transfer_id, request, task_stop);
            } catch (const std::exception& exc) {
                Complete(transfer_id, swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED,
                         swarmkit::v1::ERROR_CODE_INTERNAL,
                         "artifact transfer worker exception: " + std::string(exc.what()), {});
            } catch (...) {
                Complete(transfer_id, swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED,
                         swarmkit::v1::ERROR_CODE_INTERNAL, "artifact transfer worker exception",
                         {});
            }
            cv_.notify_all();
        }
    }

    struct Runtime {
        swarmkit::v1::ArtifactTransferStatus status;
        swarmkit::v1::ArtifactTransferStartRequest request;
        Worker worker;
        std::stop_source stop_source;
        std::string peer_key;
    };

    int max_concurrent_{1};
    int max_queued_{0};
    int max_per_peer_{1};
    int peer_bytes_per_second_{0};
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutting_down_{false};
    std::deque<std::string> queue_;
    std::unordered_map<std::string, Runtime> transfers_;
    std::vector<std::jthread> workers_;
    std::mutex bandwidth_mutex_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> peer_next_send_time_;
};

class DataServiceImpl final : public swarmkit::v1::DataService::Service {
   public:
    struct UploadSession {
        std::string upload_id;
        swarmkit::v1::ArtifactDescriptor descriptor;
        std::filesystem::path tmp_path;
        std::int64_t bytes_received{0};
        int next_chunk_index{0};
        std::int64_t created_unix_ms{0};
        std::int64_t updated_unix_ms{0};
        std::int64_t expires_unix_ms{0};
        bool committed{false};
    };

    DataServiceImpl(AgentConfig config, std::shared_ptr<internal::RuntimeCounters> counters)
        : config_(std::move(config)),
          counters_(std::move(counters)),
          artifact_store_(config_),
          peer_registry_(config_),
          data_router_(config_, &peer_registry_),
          transfer_manager_(config_.data) {}

    ~DataServiceImpl() override = default;

    grpc::Status PublishMessage(grpc::ServerContext* ctx,
                                const swarmkit::v1::PublishMessageRequest* req,
                                swarmkit::v1::PublishMessageReply* reply) override {
        if (req == nullptr || reply == nullptr || !req->has_message()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing message");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-message");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        swarmkit::v1::DataMessage message = req->message();
        if (message.topic().empty()) {
            counters_->IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message.topic is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            core::Logger::WarnFmt(
                "rpc=PublishMessage corr={} agent={} source={} target={} result=rejected "
                "reason=missing_topic",
                correlation_id, config_.agent_id, message.source_id(), message.target_id());
            return grpc::Status::OK;
        }
        if (message.payload().size() >
            static_cast<std::size_t>(config_.data.max_message_payload_bytes)) {
            counters_->IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message payload exceeds max_message_payload_bytes");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            core::Logger::WarnFmt(
                "rpc=PublishMessage corr={} agent={} topic={} source={} target={} "
                "payload_bytes={} limit={} result=rejected reason=payload_too_large",
                correlation_id, config_.agent_id, message.topic(), message.source_id(),
                message.target_id(), message.payload().size(),
                config_.data.max_message_payload_bytes);
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.message_id().empty()) {
            message.set_message_id(
                MakeCorrelationId("msg-" + SanitizedPathComponent(message.source_id())));
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            message.set_sequence(++data_sequence_);
            if (message.message_id().empty()) {
                message.set_message_id(
                    MakeCorrelationId("msg-" + SanitizedPathComponent(message.source_id())));
            }
            message_backlog_.push_back(message);
            while (static_cast<int>(message_backlog_.size()) > config_.data.message_backlog_size) {
                message_backlog_.pop_front();
            }
        }
        data_cv_.notify_all();
        counters_->IncrementDataMessagesPublished();
        core::Logger::InfoFmt(
            "rpc=PublishMessage corr={} agent={} seq={} message_id={} topic={} source={} "
            "target={} payload_bytes={} result=published",
            correlation_id, config_.agent_id, message.sequence(), message.message_id(),
            message.topic(), message.source_id(), message.target_id(), message.payload().size());

        reply->set_ok(true);
        reply->set_message("message published");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        reply->set_sequence(message.sequence());
        return grpc::Status::OK;
    }

    grpc::Status ListDataPeers(grpc::ServerContext* ctx,
                               const swarmkit::v1::DataPeerListRequest* req,
                               swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peers");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->refresh()) {
            peer_registry_.Refresh({});
        }
        peer_registry_.FillReply(correlation_id, true, "data peers listed",
                                 swarmkit::v1::ERROR_CODE_NONE, reply);
        core::Logger::InfoFmt("rpc=ListDataPeers corr={} agent={} peer_count={} refresh={}",
                              correlation_id, config_.agent_id, reply->peers_size(),
                              req->refresh());
        return grpc::Status::OK;
    }

    grpc::Status RefreshDataPeers(grpc::ServerContext* ctx,
                                  const swarmkit::v1::DataPeerRefreshRequest* req,
                                  swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peers-refresh");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::unordered_set<std::string> requested;
        for (const std::string& drone_id : req->drone_ids()) {
            requested.insert(drone_id);
        }
        peer_registry_.Refresh(requested);
        peer_registry_.FillReply(correlation_id, true, "data peers refreshed",
                                 swarmkit::v1::ERROR_CODE_NONE, reply);
        core::Logger::InfoFmt("rpc=RefreshDataPeers corr={} agent={} requested={} peer_count={}",
                              correlation_id, config_.agent_id, requested.size(),
                              reply->peers_size());
        return grpc::Status::OK;
    }

    grpc::Status UpsertDataPeer(grpc::ServerContext* ctx, const swarmkit::v1::DataPeerConfig* req,
                                swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peer-upsert");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        auto parsed = PeerRegistry::ToPeerConfig(*req);
        if (!parsed.has_value()) {
            peer_registry_.FillReply(correlation_id, false, parsed.error().message,
                                     swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, reply);
            return grpc::Status::OK;
        }
        if (const core::Result validation = parsed->Validate(); !validation.IsOk()) {
            peer_registry_.FillReply(correlation_id, false, validation.message,
                                     swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, reply);
            return grpc::Status::OK;
        }
        peer_registry_.Upsert(*parsed);
        peer_registry_.FillReply(correlation_id, true, "data peer upserted",
                                 swarmkit::v1::ERROR_CODE_NONE, reply);
        core::Logger::InfoFmt("rpc=UpsertDataPeer corr={} agent={} peer={} address={}",
                              correlation_id, config_.agent_id, parsed->drone_id, parsed->address);
        return grpc::Status::OK;
    }

    grpc::Status RemoveDataPeer(grpc::ServerContext* ctx,
                                const swarmkit::v1::DataPeerRemoveRequest* req,
                                swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peer-remove");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->drone_id().empty()) {
            peer_registry_.FillReply(correlation_id, false, "drone_id is required",
                                     swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, reply);
            return grpc::Status::OK;
        }
        const bool removed = peer_registry_.Remove(req->drone_id());
        peer_registry_.FillReply(
            correlation_id, removed, removed ? "data peer removed" : "data peer not found",
            removed ? swarmkit::v1::ERROR_CODE_NONE : swarmkit::v1::ERROR_CODE_NOT_FOUND, reply);
        return grpc::Status::OK;
    }

    grpc::Status SendMessageToDrone(grpc::ServerContext* ctx,
                                    const swarmkit::v1::PublishMessageRequest* req,
                                    swarmkit::v1::PublishMessageReply* reply) override {
        if (req == nullptr || reply == nullptr || !req->has_message()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing message");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-route-message");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        swarmkit::v1::DataMessage message = req->message();
        if (message.target_id().empty()) {
            counters_->IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message.target_id is required for routed sends");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        if (message.target_id() == config_.agent_id) {
            swarmkit::v1::PublishMessageRequest local_req;
            *local_req.mutable_message() = std::move(message);
            return PublishMessage(ctx, &local_req, reply);
        }
        const std::optional<DataPeerConfig> peer = data_router_.FindPeer(message.target_id());
        if (!peer.has_value()) {
            counters_->IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("no data peer configured for target_id=" + message.target_id());
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_NOT_FOUND);
            core::Logger::WarnFmt(
                "rpc=SendMessageToDrone corr={} agent={} target={} topic={} result=rejected "
                "reason=no_peer",
                correlation_id, config_.agent_id, message.target_id(), message.topic());
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }

        const auto route_started = std::chrono::steady_clock::now();
        core::Logger::InfoFmt(
            "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} source={} "
            "target={} message_id={} payload_bytes={} result=routing",
            correlation_id, config_.agent_id, peer->drone_id, peer->address, message.topic(),
            message.source_id(), message.target_id(), message.message_id(),
            message.payload().size());
        auto channel = data_router_.MakeChannel(*peer);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{30});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
        swarmkit::v1::PublishMessageRequest peer_req;
        *peer_req.mutable_message() = std::move(message);
        swarmkit::v1::PublishMessageReply peer_reply;
        const grpc::Status peer_status = stub->PublishMessage(&peer_ctx, peer_req, &peer_reply);
        const auto route_latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - route_started)
                                          .count();
        if (!peer_status.ok()) {
            counters_->IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("peer message delivery failed: " + peer_status.error_message());
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
            data_router_.MarkRouteResult(*peer, false, route_latency_ms,
                                         peer_status.error_message());
            core::Logger::WarnFmt(
                "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} target={} "
                "latency_ms={} result=failed error={}",
                correlation_id, config_.agent_id, peer->drone_id, peer->address,
                peer_req.message().topic(), peer_req.message().target_id(), route_latency_ms,
                peer_status.error_message());
            return grpc::Status::OK;
        }
        data_router_.MarkRouteResult(*peer, peer_reply.ok(), route_latency_ms,
                                     peer_reply.message());
        *reply = peer_reply;
        if (reply->correlation_id().empty()) {
            reply->set_correlation_id(correlation_id);
        }
        if (reply->ok()) {
            reply->set_message("message delivered to " + peer->drone_id + ": " + reply->message());
        }
        core::Logger::InfoFmt(
            "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} target={} "
            "remote_seq={} latency_ms={} result={}",
            correlation_id, config_.agent_id, peer->drone_id, peer->address,
            peer_req.message().topic(), peer_req.message().target_id(), reply->sequence(),
            route_latency_ms, reply->ok() ? "delivered" : "rejected");
        return grpc::Status::OK;
    }

    grpc::Status SubscribeMessages(grpc::ServerContext* ctx,
                                   const swarmkit::v1::MessageSubscription* req,
                                   grpc::ServerWriter<swarmkit::v1::DataMessage>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        counters_->IncrementMessageSubscribers();
        const auto decrement_subscribers = [this] { counters_->DecrementMessageSubscribers(); };
        std::uint64_t cursor = req->after_sequence();
        while (!ctx->IsCancelled()) {
            std::vector<swarmkit::v1::DataMessage> pending;
            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                data_cv_.wait_for(lock, std::chrono::milliseconds{250},
                                  [&] { return ctx->IsCancelled() || data_sequence_ > cursor; });
                const std::int64_t now_ms = NowUnixMs();
                for (const auto& message : message_backlog_) {
                    if (message.sequence() <= cursor || MessageExpired(message, now_ms) ||
                        !MessageMatchesSubscription(message, *req)) {
                        continue;
                    }
                    pending.push_back(message);
                }
            }
            for (const auto& message : pending) {
                cursor = std::max(cursor, message.sequence());
                if (!writer->Write(message)) {
                    decrement_subscribers();
                    return grpc::Status::OK;
                }
            }
        }
        decrement_subscribers();
        return grpc::Status::OK;
    }

    grpc::Status UploadArtifact(grpc::ServerContext* ctx,
                                grpc::ServerReader<swarmkit::v1::ArtifactChunk>* reader,
                                swarmkit::v1::ArtifactDescriptor* reply) override {
        if (ctx == nullptr || reader == nullptr || reply == nullptr) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reader/reply");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::filesystem::path tmp_dir = artifact_store_.TempDir();
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            const std::string detail =
                "failed to create artifact temp dir path=" + tmp_dir.string() +
                " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }

        swarmkit::v1::ArtifactDescriptor descriptor;
        std::string transfer_id;
        std::filesystem::path tmp_path;
        std::ofstream output;
        std::int64_t received_bytes = 0;
        int expected_chunk_index = 0;
        bool saw_descriptor = false;
        bool saw_final = false;
        const auto fail_upload = [&](grpc::StatusCode code, std::string_view message) {
            if (output.is_open()) {
                output.close();
            }
            if (!tmp_path.empty()) {
                std::filesystem::remove(tmp_path, fs_error);
            }
            counters_->IncrementArtifactFailures();
            return grpc::Status(code, std::string(message));
        };

        swarmkit::v1::ArtifactChunk chunk;
        while (reader->Read(&chunk)) {
            if (saw_final) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunk received after final chunk");
            }
            if (!saw_descriptor) {
                if (!chunk.has_artifact()) {
                    return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                       "first artifact chunk must include descriptor");
                }
                descriptor = chunk.artifact();
                if (descriptor.source_id().empty()) {
                    descriptor.set_source_id(config_.agent_id);
                }
                if (descriptor.created_unix_ms() == 0) {
                    descriptor.set_created_unix_ms(NowUnixMs());
                }
                transfer_id = chunk.transfer_id().empty() ? MakeCorrelationId("artifact-upload")
                                                          : chunk.transfer_id();
                tmp_path = tmp_dir / (SanitizedPathComponent(transfer_id) + ".part");
                output.open(tmp_path, std::ios::binary | std::ios::trunc);
                if (!output.is_open()) {
                    core::Logger::ErrorFmt(
                        "rpc=UploadArtifact agent={} failed to open artifact temp file path={}",
                        config_.agent_id, tmp_path.string());
                    return fail_upload(
                        grpc::StatusCode::FAILED_PRECONDITION,
                        "failed to open artifact temp file path=" + tmp_path.string());
                }
                saw_descriptor = true;
            }
            if (chunk.chunk_index() != expected_chunk_index) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunks must arrive in order with matching indexes");
            }
            if (chunk.offset() != received_bytes) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunks must arrive in order with matching offsets");
            }
            if (received_bytes + static_cast<std::int64_t>(chunk.data().size()) >
                config_.data.max_artifact_bytes) {
                return fail_upload(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                   "artifact exceeds max_artifact_bytes");
            }
            if (!WriteBytes(output,
                            std::span<const char>{chunk.data().data(), chunk.data().size()})) {
                return fail_upload(grpc::StatusCode::FAILED_PRECONDITION,
                                   "failed to write artifact temp file");
            }
            received_bytes += static_cast<std::int64_t>(chunk.data().size());
            ++expected_chunk_index;
            saw_final = saw_final || chunk.final_chunk();
        }
        if (!saw_descriptor) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "empty artifact upload");
        }
        output.flush();
        output.close();
        if (!saw_final) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "artifact upload ended before final chunk");
        }

        const std::string sha256_hex = core::internal::Sha256FileHex(tmp_path);
        if (sha256_hex.empty()) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INTERNAL, "failed to hash artifact");
        }
        if (!descriptor.sha256_hex().empty() && descriptor.sha256_hex() != sha256_hex) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::DATA_LOSS, "artifact sha256 mismatch");
        }
        descriptor.set_sha256_hex(sha256_hex);
        if (descriptor.artifact_id().empty()) {
            descriptor.set_artifact_id(sha256_hex);
        }
        descriptor.set_size_bytes(received_bytes);
        if (descriptor.filename().empty()) {
            descriptor.set_filename(descriptor.artifact_id() + ".bin");
        }

        const std::filesystem::path final_dir = artifact_store_.Root() /
                                                SanitizedPathComponent(descriptor.source_id()) /
                                                SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            const std::string detail =
                "failed to create artifact final dir path=" + final_dir.string() +
                " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            std::filesystem::remove(tmp_path, fs_error);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(tmp_path, final_path, fs_error);
        if (fs_error) {
            std::error_code exists_error;
            if (std::filesystem::exists(final_path, exists_error)) {
                const std::string existing_sha256 = core::internal::Sha256FileHex(final_path);
                if (existing_sha256 == sha256_hex) {
                    std::filesystem::remove(tmp_path, fs_error);
                    fs_error.clear();
                } else {
                    std::filesystem::remove(tmp_path, exists_error);
                    counters_->IncrementArtifactFailures();
                    const std::string detail =
                        "artifact final path exists with different sha256 path=" +
                        final_path.string() + " existing_sha256=" + existing_sha256 +
                        " incoming_sha256=" + sha256_hex;
                    core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id,
                                           detail);
                    return grpc::Status(grpc::StatusCode::ALREADY_EXISTS, detail);
                }
            }
        }
        if (fs_error) {
            const std::string detail = "failed to finalize artifact tmp_path=" + tmp_path.string() +
                                       " final_path=" + final_path.string() +
                                       " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            std::filesystem::remove(tmp_path, fs_error);
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }
        if (const core::Result persisted = ArtifactStore::PersistMetadata(descriptor, final_path);
            !persisted.IsOk()) {
            std::filesystem::remove(final_path, fs_error);
            counters_->IncrementArtifactFailures();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id,
                                   persisted.message);
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, persisted.message);
        }
        artifact_store_.Put(descriptor, final_path);
        PublishArtifactReceivedEvent(descriptor, "artifact-upload");
        *reply = descriptor;
        counters_->IncrementArtifactUploads(static_cast<std::uint64_t>(received_bytes));
        core::Logger::InfoFmt(
            "rpc=UploadArtifact agent={} artifact_id={} source={} target={} bytes={} path={} "
            "result=stored",
            config_.agent_id, descriptor.artifact_id(), descriptor.source_id(),
            descriptor.target_id(), received_bytes, final_path.string());
        return grpc::Status::OK;
    }

    grpc::Status CreateArtifactUpload(grpc::ServerContext* ctx,
                                      const swarmkit::v1::CreateArtifactUploadRequest* req,
                                      swarmkit::v1::ArtifactUploadReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-upload-create");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }

        swarmkit::v1::ArtifactDescriptor descriptor = req->artifact();
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        if (descriptor.filename().empty()) {
            descriptor.set_filename("artifact-upload.bin");
        }
        if (descriptor.size_bytes() < 0) {
            FillArtifactUploadReply(reply, false, "artifact.size_bytes must be >= 0",
                                    correlation_id, swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {},
                                    {});
            return grpc::Status::OK;
        }
        if (descriptor.size_bytes() > config_.data.max_artifact_bytes) {
            FillArtifactUploadReply(reply, false, "artifact exceeds max_artifact_bytes",
                                    correlation_id, swarmkit::v1::ERROR_CODE_REJECTED, {}, {});
            return grpc::Status::OK;
        }

        std::filesystem::path tmp_dir = artifact_store_.TempDir();
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            const std::string message =
                "failed to create artifact temp dir path=" + tmp_dir.string() +
                " error=" + fs_error.message();
            counters_->IncrementArtifactFailures();
            FillArtifactUploadReply(reply, false, message, correlation_id,
                                    swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION, {}, {});
            return grpc::Status::OK;
        }

        UploadSession session;
        session.upload_id = MakeCorrelationId("artifact-upload-session");
        session.descriptor = descriptor;
        session.tmp_path = tmp_dir / (SanitizedPathComponent(session.upload_id) + ".part");
        session.created_unix_ms = NowUnixMs();
        session.updated_unix_ms = session.created_unix_ms;
        const std::int64_t ttl_ms =
            req->ttl_ms() > 0 ? req->ttl_ms() : kDefaultArtifactUploadSessionTtlMs;
        session.expires_unix_ms = session.created_unix_ms + ttl_ms;

        std::ofstream output(session.tmp_path, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) {
            const std::string message =
                "failed to open upload session temp file path=" + session.tmp_path.string();
            counters_->IncrementArtifactFailures();
            FillArtifactUploadReply(reply, false, message, correlation_id,
                                    swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION, {}, {});
            return grpc::Status::OK;
        }
        output.close();

        const auto proto_session = ToProtoUploadSession(session);
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            upload_sessions_[session.upload_id] = std::move(session);
        }
        FillArtifactUploadReply(reply, true, "artifact upload session created", correlation_id,
                                swarmkit::v1::ERROR_CODE_NONE, proto_session, {});
        return grpc::Status::OK;
    }

    grpc::Status UploadArtifactChunk(grpc::ServerContext* ctx,
                                     const swarmkit::v1::ArtifactChunk* req,
                                     swarmkit::v1::ArtifactUploadReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-upload-chunk");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->transfer_id().empty()) {
            FillArtifactUploadReply(reply, false, "chunk.transfer_id upload_id is required",
                                    correlation_id, swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {},
                                    {});
            return grpc::Status::OK;
        }

        std::filesystem::path tmp_path;
        UploadSession session;
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            auto iter = upload_sessions_.find(req->transfer_id());
            if (iter == upload_sessions_.end()) {
                FillArtifactUploadReply(reply, false, "artifact upload session not found",
                                        correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
                return grpc::Status::OK;
            }
            if (UploadSessionExpired(iter->second)) {
                const std::filesystem::path expired_path = iter->second.tmp_path;
                upload_sessions_.erase(iter);
                std::error_code remove_error;
                std::filesystem::remove(expired_path, remove_error);
                FillArtifactUploadReply(reply, false, "artifact upload session expired",
                                        correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
                return grpc::Status::OK;
            }
            if (iter->second.committed) {
                FillArtifactUploadReply(reply, false, "artifact upload already committed",
                                        correlation_id,
                                        swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION,
                                        ToProtoUploadSession(iter->second), {});
                return grpc::Status::OK;
            }
            if (req->offset() != iter->second.bytes_received ||
                req->chunk_index() != iter->second.next_chunk_index) {
                FillArtifactUploadReply(
                    reply, false, "artifact upload chunk offset/index does not match session",
                    correlation_id, swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION,
                    ToProtoUploadSession(iter->second), {});
                return grpc::Status::OK;
            }
            const std::int64_t next_size =
                iter->second.bytes_received + static_cast<std::int64_t>(req->data().size());
            if (next_size > config_.data.max_artifact_bytes ||
                (iter->second.descriptor.size_bytes() > 0 &&
                 next_size > iter->second.descriptor.size_bytes())) {
                FillArtifactUploadReply(reply, false, "artifact exceeds configured/declared size",
                                        correlation_id, swarmkit::v1::ERROR_CODE_REJECTED,
                                        ToProtoUploadSession(iter->second), {});
                return grpc::Status::OK;
            }
            tmp_path = iter->second.tmp_path;
            session = iter->second;
        }

        std::ofstream output(tmp_path, std::ios::binary | std::ios::app);
        if (!output.is_open()) {
            counters_->IncrementArtifactFailures();
            FillArtifactUploadReply(
                reply, false, "failed to open upload session temp file path=" + tmp_path.string(),
                correlation_id, swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION, {}, {});
            return grpc::Status::OK;
        }
        if (!WriteBytes(output, std::span<const char>{req->data().data(), req->data().size()})) {
            counters_->IncrementArtifactFailures();
            FillArtifactUploadReply(reply, false, "failed to write upload session chunk",
                                    correlation_id, swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION,
                                    {}, {});
            return grpc::Status::OK;
        }
        output.close();

        swarmkit::v1::ArtifactUploadSession proto_session;
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            auto iter = upload_sessions_.find(req->transfer_id());
            if (iter == upload_sessions_.end()) {
                FillArtifactUploadReply(reply, false, "artifact upload session not found",
                                        correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
                return grpc::Status::OK;
            }
            iter->second.bytes_received += static_cast<std::int64_t>(req->data().size());
            ++iter->second.next_chunk_index;
            iter->second.updated_unix_ms = NowUnixMs();
            proto_session = ToProtoUploadSession(iter->second);
        }
        FillArtifactUploadReply(reply, true, "artifact upload chunk accepted", correlation_id,
                                swarmkit::v1::ERROR_CODE_NONE, proto_session, {});
        return grpc::Status::OK;
    }

    grpc::Status GetUploadStatus(grpc::ServerContext* ctx,
                                 const swarmkit::v1::ArtifactUploadRequest* req,
                                 swarmkit::v1::ArtifactUploadReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-upload-status");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto session = FindUploadSession(req->upload_id(), true);
        if (!session.has_value()) {
            FillArtifactUploadReply(reply, false, "artifact upload session not found",
                                    correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
            return grpc::Status::OK;
        }
        FillArtifactUploadReply(reply, true, "artifact upload session found", correlation_id,
                                swarmkit::v1::ERROR_CODE_NONE, ToProtoUploadSession(*session), {});
        return grpc::Status::OK;
    }

    grpc::Status CommitArtifactUpload(grpc::ServerContext* ctx,
                                      const swarmkit::v1::ArtifactUploadRequest* req,
                                      swarmkit::v1::ArtifactUploadReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-upload-commit");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto session = FindUploadSession(req->upload_id(), true);
        if (!session.has_value()) {
            FillArtifactUploadReply(reply, false, "artifact upload session not found",
                                    correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
            return grpc::Status::OK;
        }
        if (session->descriptor.size_bytes() > 0 &&
            session->bytes_received != session->descriptor.size_bytes()) {
            FillArtifactUploadReply(reply, false, "artifact upload is incomplete", correlation_id,
                                    swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION,
                                    ToProtoUploadSession(*session), {});
            return grpc::Status::OK;
        }

        ArtifactTransferOutcome outcome = CommitUploadSession(*session);
        if (!outcome.ok) {
            counters_->IncrementArtifactFailures();
            FillArtifactUploadReply(reply, false, outcome.message, correlation_id,
                                    outcome.error_code, ToProtoUploadSession(*session), {});
            return grpc::Status::OK;
        }
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            upload_sessions_.erase(req->upload_id());
        }
        FillArtifactUploadReply(reply, true, "artifact upload committed", correlation_id,
                                swarmkit::v1::ERROR_CODE_NONE, {}, outcome.descriptor);
        return grpc::Status::OK;
    }

    grpc::Status CancelUpload(grpc::ServerContext* ctx,
                              const swarmkit::v1::ArtifactUploadRequest* req,
                              swarmkit::v1::ArtifactUploadReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-upload-cancel");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::filesystem::path tmp_path;
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            auto iter = upload_sessions_.find(req->upload_id());
            if (iter == upload_sessions_.end()) {
                FillArtifactUploadReply(reply, false, "artifact upload session not found",
                                        correlation_id, swarmkit::v1::ERROR_CODE_NOT_FOUND, {}, {});
                return grpc::Status::OK;
            }
            tmp_path = iter->second.tmp_path;
            upload_sessions_.erase(iter);
        }
        std::error_code remove_error;
        std::filesystem::remove(tmp_path, remove_error);
        FillArtifactUploadReply(reply, true, "artifact upload cancelled", correlation_id,
                                swarmkit::v1::ERROR_CODE_NONE, {}, {});
        return grpc::Status::OK;
    }

    grpc::Status SendArtifactToDrone(grpc::ServerContext* ctx,
                                     grpc::ServerReader<swarmkit::v1::ArtifactChunk>* reader,
                                     swarmkit::v1::ArtifactDescriptor* reply) override {
        if (ctx == nullptr || reader == nullptr || reply == nullptr) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reader/reply");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-route");

        swarmkit::v1::ArtifactChunk chunk;
        if (!reader->Read(&chunk)) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "empty artifact send");
        }
        if (!chunk.has_artifact()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "first artifact chunk must include descriptor");
        }
        swarmkit::v1::ArtifactDescriptor descriptor = chunk.artifact();
        if (descriptor.target_id().empty()) {
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} result=rejected reason=missing_target",
                correlation_id, config_.agent_id);
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "artifact.target_id is required for routed sends");
        }
        if (descriptor.target_id() == config_.agent_id) {
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} target={} result=rejected "
                "reason=local_target",
                correlation_id, config_.agent_id, descriptor.target_id());
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "routed artifact target_id points to local agent");
        }
        const std::optional<DataPeerConfig> peer = data_router_.FindPeer(descriptor.target_id());
        if (!peer.has_value()) {
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} target={} result=rejected "
                "reason=no_peer",
                correlation_id, config_.agent_id, descriptor.target_id());
            return grpc::Status(
                grpc::StatusCode::NOT_FOUND,
                "route failed before upload: no data peer configured for target_id=" +
                    descriptor.target_id());
        }
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        *chunk.mutable_artifact() = descriptor;

        auto channel = data_router_.MakeChannel(*peer);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{60});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
        auto writer = stub->UploadArtifact(&peer_ctx, reply);
        auto bytes_forwarded = static_cast<std::uint64_t>(chunk.data().size());
        bool wrote_all = writer->Write(chunk);
        while (wrote_all && reader->Read(&chunk)) {
            bytes_forwarded += static_cast<std::uint64_t>(chunk.data().size());
            wrote_all = writer->Write(chunk);
        }
        writer->WritesDone();
        const grpc::Status peer_status = writer->Finish();
        if (!wrote_all && peer_status.ok()) {
            counters_->IncrementArtifactFailures();
            data_router_.MarkRouteResult(*peer, false, 0, "peer stream closed");
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} result=failed "
                "error=peer_stream_closed",
                correlation_id, config_.agent_id, peer->drone_id, peer->address);
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "peer artifact stream closed before all chunks were accepted");
        }
        if (!peer_status.ok()) {
            counters_->IncrementArtifactFailures();
            data_router_.MarkRouteResult(*peer, false, 0, peer_status.error_message());
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} result=failed "
                "error={}",
                correlation_id, config_.agent_id, peer->drone_id, peer->address,
                peer_status.error_message());
            return grpc::Status(peer_status.error_code(),
                                "peer artifact delivery failed: " + peer_status.error_message());
        }
        data_router_.MarkRouteResult(*peer, true, 0, "artifact delivered");
        counters_->IncrementArtifactDownloads(bytes_forwarded);
        core::Logger::InfoFmt(
            "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} artifact_id={} "
            "bytes={} result=delivered",
            correlation_id, config_.agent_id, peer->drone_id, peer->address, reply->artifact_id(),
            bytes_forwarded);
        return grpc::Status::OK;
    }

    grpc::Status StartArtifactTransfer(grpc::ServerContext* ctx,
                                       const swarmkit::v1::ArtifactTransferStartRequest* req,
                                       swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-transfer");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->source_path().empty()) {
            FillArtifactTransferReply(reply, false, "source_path is required", correlation_id,
                                      swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {});
            return grpc::Status::OK;
        }
        if (req->route_to_target() && req->artifact().target_id().empty()) {
            FillArtifactTransferReply(
                reply, false, "artifact.target_id is required for routed transfer", correlation_id,
                swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {});
            return grpc::Status::OK;
        }
        std::error_code fs_error;
        const auto file_size = std::filesystem::file_size(req->source_path(), fs_error);
        if (fs_error) {
            FillArtifactTransferReply(reply, false,
                                      "source_path cannot be read path=" + req->source_path() +
                                          " error=" + fs_error.message(),
                                      correlation_id, swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT,
                                      {});
            return grpc::Status::OK;
        }

        swarmkit::v1::ArtifactTransferStartRequest request = *req;
        ArtifactTransferManager::StartResult start_result = transfer_manager_.Start(
            request, static_cast<std::int64_t>(file_size),
            [this](std::string transfer_id,
                   const swarmkit::v1::ArtifactTransferStartRequest& request,
                   const std::stop_token& stop_token) {
                RunArtifactTransfer(std::move(transfer_id), request, stop_token);
            });
        const swarmkit::v1::ArtifactTransferStatus& status = start_result.status;
        const std::string transfer_id = status.transfer_id();
        if (!start_result.accepted) {
            core::Logger::WarnFmt(
                "rpc=StartArtifactTransfer corr={} agent={} transfer_id={} source_path={} "
                "target={} route={} bytes={} result=rejected error={}",
                correlation_id, config_.agent_id, transfer_id, req->source_path(),
                req->artifact().target_id(), req->route_to_target(), file_size, status.message());
            FillArtifactTransferReply(reply, false, status.message(), correlation_id,
                                      status.error_code(), status);
            return grpc::Status::OK;
        }
        core::Logger::InfoFmt(
            "rpc=StartArtifactTransfer corr={} agent={} transfer_id={} source_path={} "
            "target={} route={} bytes={} result=queued",
            correlation_id, config_.agent_id, transfer_id, req->source_path(),
            req->artifact().target_id(), req->route_to_target(), file_size);
        FillArtifactTransferReply(reply, true, "artifact transfer queued", correlation_id,
                                  swarmkit::v1::ERROR_CODE_NONE, status);
        return grpc::Status::OK;
    }

    grpc::Status GetArtifactTransfer(grpc::ServerContext* ctx,
                                     const swarmkit::v1::ArtifactTransferRequest* req,
                                     swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-transfer-get");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto status = transfer_manager_.Find(req->transfer_id());
        if (!status.has_value()) {
            FillArtifactTransferReply(reply, false, "artifact transfer not found", correlation_id,
                                      swarmkit::v1::ERROR_CODE_NOT_FOUND, {});
            return grpc::Status::OK;
        }
        FillArtifactTransferReply(reply, true, status->message(), correlation_id,
                                  status->error_code(), *status);
        return grpc::Status::OK;
    }

    grpc::Status CancelArtifactTransfer(grpc::ServerContext* ctx,
                                        const swarmkit::v1::ArtifactTransferRequest* req,
                                        swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id =
            ResolveCorrelationId(ctx, "", "artifact-transfer-cancel");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto status = transfer_manager_.Cancel(req->transfer_id());
        if (!status.has_value()) {
            FillArtifactTransferReply(reply, false, "artifact transfer not found", correlation_id,
                                      swarmkit::v1::ERROR_CODE_NOT_FOUND, {});
            return grpc::Status::OK;
        }
        core::Logger::InfoFmt("rpc=CancelArtifactTransfer corr={} agent={} transfer_id={}",
                              correlation_id, config_.agent_id, req->transfer_id());
        FillArtifactTransferReply(reply, true, status->message(), correlation_id,
                                  status->error_code(), *status);
        return grpc::Status::OK;
    }

    grpc::Status ListArtifacts(grpc::ServerContext* ctx,
                               const swarmkit::v1::ArtifactListRequest* req,
                               swarmkit::v1::ArtifactListReply* reply) override {
        if (ctx == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-list");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const std::string source_id = req == nullptr ? "" : req->source_id();
        const std::string target_id = req == nullptr ? "" : req->target_id();
        const bool include_expired = req != nullptr && req->include_expired();
        const int requested_page_size = req == nullptr ? 0 : req->page_size();
        const int page_size = requested_page_size <= 0 ? 100 : std::min(requested_page_size, 1000);
        std::size_t start_index = 0;
        if (req != nullptr && !req->page_token().empty()) {
            try {
                start_index = static_cast<std::size_t>(std::stoull(req->page_token()));
            } catch (const std::exception&) {
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                    "page_token must be a non-negative integer offset");
            }
        }
        const ArtifactStore::ListPage page =
            artifact_store_.List(source_id, target_id, include_expired, start_index,
                                 static_cast<std::size_t>(page_size));
        reply->set_total_count(page.total_count);
        for (const auto& artifact : page.artifacts) {
            *reply->add_artifacts() = artifact;
        }
        reply->set_next_page_token(page.next_page_token);
        reply->set_correlation_id(correlation_id);
        core::Logger::InfoFmt(
            "rpc=ListArtifacts corr={} agent={} count={} total={} source_filter={} "
            "target_filter={} page_size={} next_page_token={}",
            correlation_id, config_.agent_id, reply->artifacts_size(), reply->total_count(),
            source_id, target_id, page_size, reply->next_page_token());
        return grpc::Status::OK;
    }

    grpc::Status GetArtifact(grpc::ServerContext* ctx, const swarmkit::v1::ArtifactRequest* req,
                             swarmkit::v1::ArtifactReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-info");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->artifact_id().empty()) {
            reply->set_ok(false);
            reply->set_message("artifact_id is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        const auto stored = artifact_store_.FindIncludingExpired(req->artifact_id());
        if (!stored.has_value()) {
            reply->set_ok(false);
            reply->set_message("artifact not found");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_NOT_FOUND);
            return grpc::Status::OK;
        }
        if (ArtifactExpired(stored->descriptor, NowUnixMs())) {
            artifact_store_.Remove(req->artifact_id());
            reply->set_ok(false);
            reply->set_message("artifact expired");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_NOT_FOUND);
            return grpc::Status::OK;
        }
        *reply->mutable_artifact() = stored->descriptor;
        reply->set_ok(true);
        reply->set_message("artifact found");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        return grpc::Status::OK;
    }

    grpc::Status DownloadArtifact(
        grpc::ServerContext* ctx, const swarmkit::v1::ArtifactRequest* req,
        grpc::ServerWriter<swarmkit::v1::ArtifactChunk>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto stored = artifact_store_.FindIncludingExpired(req->artifact_id());
        if (!stored.has_value()) {
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} result=failed reason=not_found",
                config_.agent_id, req->artifact_id());
            return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact not found");
        }
        swarmkit::v1::ArtifactDescriptor descriptor = stored->descriptor;
        std::filesystem::path storage_path = stored->storage_path;
        if (ArtifactExpired(descriptor, NowUnixMs())) {
            artifact_store_.Remove(req->artifact_id());
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} result=failed reason=expired",
                config_.agent_id, req->artifact_id());
            return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact expired");
        }
        std::ifstream input(storage_path, std::ios::binary);
        if (!input.is_open()) {
            counters_->IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} path={} result=failed "
                "reason=file_not_found",
                config_.agent_id, req->artifact_id(), storage_path.string());
            return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact file not found");
        }
        if (descriptor.size_bytes() == 0) {
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id("download-" + descriptor.artifact_id());
            *chunk.mutable_artifact() = descriptor;
            chunk.set_offset(0);
            chunk.set_chunk_index(0);
            chunk.set_final_chunk(true);
            static_cast<void>(writer->Write(chunk));
            counters_->IncrementArtifactDownloads(0);
            core::Logger::InfoFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} bytes=0 result=sent",
                config_.agent_id, descriptor.artifact_id());
            return grpc::Status::OK;
        }
        const std::int64_t requested_offset = std::max<std::int64_t>(0, req->offset_bytes());
        if (requested_offset > descriptor.size_bytes()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "offset_bytes is beyond artifact size");
        }
        if (requested_offset == descriptor.size_bytes()) {
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id("download-" + descriptor.artifact_id());
            *chunk.mutable_artifact() = descriptor;
            chunk.set_offset(requested_offset);
            chunk.set_chunk_index(0);
            chunk.set_final_chunk(true);
            static_cast<void>(writer->Write(chunk));
            counters_->IncrementArtifactDownloads(0);
            return grpc::Status::OK;
        }
        if (requested_offset > 0) {
            input.seekg(requested_offset, std::ios::beg);
            if (!input.good()) {
                counters_->IncrementArtifactFailures();
                return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                    "failed to seek artifact file");
            }
        }
        const std::size_t chunk_size =
            std::max<std::size_t>(1, static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t offset = requested_offset;
        int chunk_index = 0;
        while (!ctx->IsCancelled() && input.good()) {
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id("download-" + descriptor.artifact_id());
            if (chunk_index == 0) {
                *chunk.mutable_artifact() = descriptor;
            }
            chunk.set_offset(offset);
            chunk.set_chunk_index(chunk_index);
            chunk.set_data(buffer.data(), static_cast<std::size_t>(read));
            offset += read;
            chunk.set_final_chunk(offset >= descriptor.size_bytes());
            if (!writer->Write(chunk)) {
                counters_->IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
                return grpc::Status::OK;
            }
            ++chunk_index;
        }
        counters_->IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
        core::Logger::InfoFmt("rpc=DownloadArtifact agent={} artifact_id={} bytes={} result=sent",
                              config_.agent_id, descriptor.artifact_id(), offset);
        return grpc::Status::OK;
    }

    grpc::Status AnnounceArtifact(grpc::ServerContext* ctx,
                                  const swarmkit::v1::ArtifactDescriptor* req,
                                  swarmkit::v1::ArtifactReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-announce");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_->IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->artifact_id().empty()) {
            counters_->IncrementArtifactFailures();
            reply->set_ok(false);
            reply->set_message("artifact_id is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            core::Logger::WarnFmt(
                "rpc=AnnounceArtifact corr={} agent={} result=rejected reason=missing_artifact_id",
                correlation_id, config_.agent_id);
            return grpc::Status::OK;
        }
        swarmkit::v1::ArtifactDescriptor descriptor = *req;
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        artifact_store_.Put(descriptor, {});
        *reply->mutable_artifact() = descriptor;
        reply->set_ok(true);
        reply->set_message("artifact announced");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        core::Logger::InfoFmt(
            "rpc=AnnounceArtifact corr={} agent={} artifact_id={} source={} target={} "
            "result=announced",
            correlation_id, config_.agent_id, reply->artifact().artifact_id(),
            reply->artifact().source_id(), reply->artifact().target_id());
        return grpc::Status::OK;
    }

   private:
    void PublishArtifactReceivedEvent(const swarmkit::v1::ArtifactDescriptor& descriptor,
                                      std::string_view origin = "artifact-transfer") {
        swarmkit::v1::DataMessage event;
        event.set_message_id(MakeCorrelationId("artifact-received"));
        event.set_source_id(config_.agent_id);
        event.set_topic("swarmkit.artifact.received");
        event.set_unix_time_ms(NowUnixMs());
        event.set_ttl_ms(descriptor.ttl_ms());
        auto& labels = *event.mutable_labels();
        labels["artifact_id"] = descriptor.artifact_id();
        labels["artifact_source_id"] = descriptor.source_id();
        labels["source_id"] = descriptor.source_id();
        labels["target_id"] = descriptor.target_id();
        labels["content_type"] = descriptor.content_type();
        labels["filename"] = descriptor.filename();
        labels["size_bytes"] = std::to_string(descriptor.size_bytes());
        labels["sha256_hex"] = descriptor.sha256_hex();
        labels["origin"] = std::string(origin);
        for (const auto& [key, value] : descriptor.labels()) {
            labels["artifact.label." + key] = value;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            event.set_sequence(++data_sequence_);
            message_backlog_.push_back(event);
            while (static_cast<int>(message_backlog_.size()) > config_.data.message_backlog_size) {
                message_backlog_.pop_front();
            }
        }
        data_cv_.notify_all();
        counters_->IncrementDataMessagesPublished();
        core::Logger::InfoFmt(
            "event=artifact.received agent={} artifact_id={} artifact_source={} target={} "
            "content_type={} bytes={} origin={} result=published",
            config_.agent_id, descriptor.artifact_id(), descriptor.source_id(),
            descriptor.target_id(), descriptor.content_type(), descriptor.size_bytes(), origin);
    }

    static void FillArtifactTransferReply(swarmkit::v1::ArtifactTransferReply* reply, bool success,
                                          std::string_view message, std::string_view correlation_id,
                                          swarmkit::v1::ErrorCode error_code,
                                          const swarmkit::v1::ArtifactTransferStatus& status) {
        if (reply == nullptr) {
            return;
        }
        reply->set_ok(success);
        reply->set_message(std::string(message));
        reply->set_correlation_id(std::string(correlation_id));
        reply->set_error_code(error_code);
        reply->set_debug_message(std::string(message));
        if (!status.transfer_id().empty()) {
            *reply->mutable_transfer() = status;
        }
    }

    [[nodiscard]] static bool UploadSessionExpired(const UploadSession& session) {
        return session.expires_unix_ms > 0 && NowUnixMs() > session.expires_unix_ms;
    }

    [[nodiscard]] static swarmkit::v1::ArtifactUploadSession ToProtoUploadSession(
        const UploadSession& session) {
        swarmkit::v1::ArtifactUploadSession proto;
        proto.set_upload_id(session.upload_id);
        *proto.mutable_artifact() = session.descriptor;
        proto.set_bytes_received(session.bytes_received);
        proto.set_next_chunk_index(session.next_chunk_index);
        proto.set_created_unix_ms(session.created_unix_ms);
        proto.set_updated_unix_ms(session.updated_unix_ms);
        proto.set_expires_unix_ms(session.expires_unix_ms);
        proto.set_committed(session.committed);
        return proto;
    }

    static void FillArtifactUploadReply(swarmkit::v1::ArtifactUploadReply* reply, bool success,
                                        std::string_view message, std::string_view correlation_id,
                                        swarmkit::v1::ErrorCode error_code,
                                        const swarmkit::v1::ArtifactUploadSession& session,
                                        const swarmkit::v1::ArtifactDescriptor& artifact) {
        if (reply == nullptr) {
            return;
        }
        reply->set_ok(success);
        reply->set_message(std::string(message));
        reply->set_correlation_id(std::string(correlation_id));
        reply->set_error_code(error_code);
        reply->set_debug_message(std::string(message));
        if (!session.upload_id().empty()) {
            *reply->mutable_upload() = session;
        }
        if (!artifact.artifact_id().empty()) {
            *reply->mutable_artifact() = artifact;
        }
    }

    [[nodiscard]] std::optional<UploadSession> FindUploadSession(std::string_view upload_id,
                                                                 bool cleanup_expired) {
        if (upload_id.empty()) {
            return std::nullopt;
        }
        std::filesystem::path expired_path;
        {
            std::lock_guard<std::mutex> lock(upload_mutex_);
            auto iter = upload_sessions_.find(std::string(upload_id));
            if (iter == upload_sessions_.end()) {
                return std::nullopt;
            }
            if (UploadSessionExpired(iter->second)) {
                expired_path = iter->second.tmp_path;
                if (cleanup_expired) {
                    upload_sessions_.erase(iter);
                } else {
                    return iter->second;
                }
            } else {
                return iter->second;
            }
        }
        if (!expired_path.empty()) {
            std::error_code remove_error;
            std::filesystem::remove(expired_path, remove_error);
        }
        return std::nullopt;
    }

    [[nodiscard]] ArtifactTransferOutcome CommitUploadSession(const UploadSession& session) {
        ArtifactTransferOutcome outcome;
        swarmkit::v1::ArtifactDescriptor descriptor = session.descriptor;
        const std::string sha256_hex = core::internal::Sha256FileHex(session.tmp_path);
        if (sha256_hex.empty()) {
            outcome.message = "failed to hash artifact";
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        if (!descriptor.sha256_hex().empty() && descriptor.sha256_hex() != sha256_hex) {
            outcome.message = "artifact sha256 mismatch";
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        descriptor.set_sha256_hex(sha256_hex);
        if (descriptor.artifact_id().empty()) {
            descriptor.set_artifact_id(sha256_hex);
        }
        descriptor.set_size_bytes(session.bytes_received);
        if (descriptor.filename().empty()) {
            descriptor.set_filename(descriptor.artifact_id() + ".bin");
        }

        std::error_code fs_error;
        const std::filesystem::path final_dir = artifact_store_.Root() /
                                                SanitizedPathComponent(descriptor.source_id()) /
                                                SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            outcome.message = "failed to create artifact final dir path=" + final_dir.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(session.tmp_path, final_path, fs_error);
        if (fs_error) {
            std::error_code exists_error;
            if (std::filesystem::exists(final_path, exists_error) &&
                core::internal::Sha256FileHex(final_path) == sha256_hex) {
                std::filesystem::remove(session.tmp_path, exists_error);
                fs_error.clear();
            }
        }
        if (fs_error) {
            outcome.message = "failed to finalize artifact tmp_path=" + session.tmp_path.string() +
                              " final_path=" + final_path.string() + " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        if (const core::Result persisted = ArtifactStore::PersistMetadata(descriptor, final_path);
            !persisted.IsOk()) {
            std::filesystem::remove(final_path, fs_error);
            outcome.message = persisted.message;
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }

        artifact_store_.Put(descriptor, final_path);
        PublishArtifactReceivedEvent(descriptor, "artifact-upload-session");
        counters_->IncrementArtifactUploads(static_cast<std::uint64_t>(session.bytes_received));
        outcome.ok = true;
        outcome.error_code = swarmkit::v1::ERROR_CODE_NONE;
        outcome.message = "artifact upload committed";
        outcome.descriptor = descriptor;
        return outcome;
    }

    [[nodiscard]] swarmkit::v1::ArtifactDescriptor PrepareArtifactDescriptorForPath(
        const std::filesystem::path& source_path, swarmkit::v1::ArtifactDescriptor descriptor,
        std::int64_t file_size) const {
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        if (descriptor.filename().empty()) {
            descriptor.set_filename(source_path.filename().string());
        }
        if (descriptor.content_type().empty()) {
            descriptor.set_content_type("application/octet-stream");
        }
        descriptor.set_size_bytes(file_size);
        return descriptor;
    }

    [[nodiscard]] ArtifactTransferOutcome StoreArtifactFileLocally(
        std::string_view transfer_id, const std::filesystem::path& source_path,
        swarmkit::v1::ArtifactDescriptor descriptor, const std::stop_token& stop_token) {
        ArtifactTransferOutcome outcome;
        std::ifstream input(source_path, std::ios::binary);
        if (!input.is_open()) {
            outcome.message = "failed to open source_path=" + source_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
            return outcome;
        }
        std::filesystem::path tmp_dir = artifact_store_.TempDir();
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            outcome.message = "failed to create artifact temp dir path=" + tmp_dir.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        const std::filesystem::path tmp_path =
            tmp_dir / (SanitizedPathComponent(std::string(transfer_id)) + ".part");
        std::ofstream output(tmp_path, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) {
            outcome.message = "failed to open artifact temp file path=" + tmp_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }

        const std::size_t chunk_size =
            std::max<std::size_t>(1, static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t bytes = 0;
        while (input.good()) {
            if (stop_token.stop_requested()) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "artifact transfer cancelled";
                outcome.error_code = swarmkit::v1::ERROR_CODE_CANCELLED;
                return outcome;
            }
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            if (bytes + static_cast<std::int64_t>(read) > config_.data.max_artifact_bytes) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "artifact exceeds max_artifact_bytes";
                outcome.error_code = swarmkit::v1::ERROR_CODE_REJECTED;
                return outcome;
            }
            if (!WriteBytes(output,
                            std::span<const char>{buffer.data(), static_cast<std::size_t>(read)})) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "failed to write artifact temp file path=" + tmp_path.string();
                outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
                return outcome;
            }
            bytes += static_cast<std::int64_t>(read);
            transfer_manager_.Update(transfer_id,
                                     [bytes](swarmkit::v1::ArtifactTransferStatus* status) {
                                         status->set_bytes_transferred(bytes);
                                     });
        }
        output.flush();
        output.close();

        const std::string sha256_hex = core::internal::Sha256FileHex(tmp_path);
        if (sha256_hex.empty()) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to hash artifact";
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        if (!descriptor.sha256_hex().empty() && descriptor.sha256_hex() != sha256_hex) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "artifact sha256 mismatch";
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        descriptor.set_sha256_hex(sha256_hex);
        if (descriptor.artifact_id().empty()) {
            descriptor.set_artifact_id(sha256_hex);
        }

        const std::filesystem::path final_dir = artifact_store_.Root() /
                                                SanitizedPathComponent(descriptor.source_id()) /
                                                SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to create artifact final dir path=" + final_dir.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(tmp_path, final_path, fs_error);
        if (fs_error) {
            std::error_code exists_error;
            if (std::filesystem::exists(final_path, exists_error) &&
                core::internal::Sha256FileHex(final_path) == sha256_hex) {
                std::filesystem::remove(tmp_path, exists_error);
                fs_error.clear();
            }
        }
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to finalize artifact tmp_path=" + tmp_path.string() +
                              " final_path=" + final_path.string() + " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }
        if (const core::Result persisted = ArtifactStore::PersistMetadata(descriptor, final_path);
            !persisted.IsOk()) {
            std::filesystem::remove(final_path, fs_error);
            outcome.message = persisted.message;
            outcome.error_code = swarmkit::v1::ERROR_CODE_FAILED_PRECONDITION;
            return outcome;
        }

        artifact_store_.Put(descriptor, final_path);
        PublishArtifactReceivedEvent(descriptor);
        counters_->IncrementArtifactUploads(static_cast<std::uint64_t>(bytes));
        outcome.ok = true;
        outcome.error_code = swarmkit::v1::ERROR_CODE_NONE;
        outcome.message = "artifact stored";
        outcome.descriptor = descriptor;
        return outcome;
    }

    [[nodiscard]] ArtifactTransferOutcome RouteArtifactFileToPeer(
        std::string_view transfer_id, const std::filesystem::path& source_path,
        const swarmkit::v1::ArtifactDescriptor& descriptor, int chunk_bytes,
        const std::stop_token& stop_token) {
        ArtifactTransferOutcome outcome;
        const std::optional<DataPeerConfig> peer = data_router_.FindPeer(descriptor.target_id());
        if (!peer.has_value()) {
            outcome.message = "route failed before upload: no data peer configured for target_id=" +
                              descriptor.target_id();
            outcome.error_code = swarmkit::v1::ERROR_CODE_NOT_FOUND;
            return outcome;
        }
        std::ifstream input(source_path, std::ios::binary);
        if (!input.is_open()) {
            outcome.message = "failed to open source_path=" + source_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
            return outcome;
        }

        const auto route_started = std::chrono::steady_clock::now();
        auto channel = data_router_.MakeChannel(*peer);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{300});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), std::string(transfer_id));
        swarmkit::v1::ArtifactDescriptor peer_reply;
        auto writer = stub->UploadArtifact(&peer_ctx, &peer_reply);

        const std::size_t chunk_size = std::max<std::size_t>(
            1, chunk_bytes > 0 ? static_cast<std::size_t>(chunk_bytes)
                               : static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t offset = 0;
        int chunk_index = 0;
        bool wrote_all = true;
        while (input.good()) {
            if (stop_token.stop_requested()) {
                peer_ctx.TryCancel();
                writer->WritesDone();
                static_cast<void>(writer->Finish());
                outcome.message = "artifact transfer cancelled";
                outcome.error_code = swarmkit::v1::ERROR_CODE_CANCELLED;
                return outcome;
            }
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            if (!transfer_manager_.ThrottlePeerBandwidth(
                    peer->drone_id, static_cast<std::size_t>(read), stop_token)) {
                peer_ctx.TryCancel();
                writer->WritesDone();
                static_cast<void>(writer->Finish());
                outcome.message = "artifact transfer cancelled";
                outcome.error_code = swarmkit::v1::ERROR_CODE_CANCELLED;
                return outcome;
            }
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id(std::string(transfer_id));
            if (chunk_index == 0) {
                *chunk.mutable_artifact() = descriptor;
            }
            chunk.set_offset(offset);
            chunk.set_chunk_index(chunk_index);
            chunk.set_data(buffer.data(), static_cast<std::size_t>(read));
            offset += static_cast<std::int64_t>(read);
            chunk.set_final_chunk(offset >= descriptor.size_bytes());
            wrote_all = writer->Write(chunk);
            if (!wrote_all) {
                break;
            }
            transfer_manager_.Update(transfer_id,
                                     [offset](swarmkit::v1::ArtifactTransferStatus* status) {
                                         status->set_bytes_transferred(offset);
                                     });
            ++chunk_index;
        }
        writer->WritesDone();
        const grpc::Status peer_status = writer->Finish();
        const auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - route_started)
                                    .count();
        if (!wrote_all && peer_status.ok()) {
            data_router_.MarkRouteResult(*peer, false, latency_ms, "peer stream closed");
            outcome.message = "peer artifact stream closed before all chunks were accepted";
            outcome.error_code = swarmkit::v1::ERROR_CODE_UNAVAILABLE;
            return outcome;
        }
        if (!peer_status.ok()) {
            data_router_.MarkRouteResult(*peer, false, latency_ms, peer_status.error_message());
            outcome.message = "peer artifact delivery failed: " + peer_status.error_message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_UNAVAILABLE;
            return outcome;
        }
        data_router_.MarkRouteResult(*peer, true, latency_ms, "artifact delivered");
        counters_->IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
        outcome.ok = true;
        outcome.error_code = swarmkit::v1::ERROR_CODE_NONE;
        outcome.message = "artifact delivered";
        outcome.descriptor = peer_reply;
        return outcome;
    }

    void RunArtifactTransfer(std::string transfer_id,
                             const swarmkit::v1::ArtifactTransferStartRequest& request,
                             const std::stop_token& stop_token) {
        transfer_manager_.Update(transfer_id, [](swarmkit::v1::ArtifactTransferStatus* status) {
            status->set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_RUNNING);
            status->set_message("running");
        });

        std::error_code fs_error;
        const auto file_size = std::filesystem::file_size(request.source_path(), fs_error);
        swarmkit::v1::ArtifactDescriptor descriptor =
            PrepareArtifactDescriptorForPath(request.source_path(), request.artifact(),
                                             fs_error ? 0 : static_cast<std::int64_t>(file_size));
        transfer_manager_.Update(transfer_id,
                                 [&descriptor](swarmkit::v1::ArtifactTransferStatus* status) {
                                     *status->mutable_artifact() = descriptor;
                                     status->set_bytes_total(descriptor.size_bytes());
                                 });
        ArtifactTransferOutcome outcome;
        if (fs_error) {
            outcome.message = "source_path cannot be read path=" + request.source_path() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
        } else if (request.route_to_target()) {
            outcome = RouteArtifactFileToPeer(transfer_id, request.source_path(), descriptor,
                                              request.chunk_bytes(), stop_token);
        } else {
            outcome = StoreArtifactFileLocally(transfer_id, request.source_path(), descriptor,
                                               stop_token);
        }

        const bool cancelled = outcome.error_code == swarmkit::v1::ERROR_CODE_CANCELLED;
        swarmkit::v1::ArtifactTransferState completion_state =
            swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED;
        if (outcome.ok) {
            completion_state = swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED;
        } else if (cancelled) {
            completion_state = swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED;
        }
        transfer_manager_.Complete(transfer_id, completion_state, outcome.error_code,
                                   outcome.message, outcome.descriptor);
        if (outcome.ok) {
            core::Logger::InfoFmt(
                "artifact_transfer agent={} transfer_id={} artifact_id={} bytes={} result={}",
                config_.agent_id, transfer_id, outcome.descriptor.artifact_id(),
                outcome.descriptor.size_bytes(),
                request.route_to_target() ? "delivered" : "stored");
        } else {
            core::Logger::WarnFmt(
                "artifact_transfer agent={} transfer_id={} source_path={} target={} result={} "
                "error={}",
                config_.agent_id, transfer_id, request.source_path(),
                request.artifact().target_id(), cancelled ? "cancelled" : "failed",
                outcome.message);
        }
    }

    AgentConfig config_;
    std::shared_ptr<internal::RuntimeCounters> counters_;
    mutable std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::deque<swarmkit::v1::DataMessage> message_backlog_;
    std::mutex upload_mutex_;
    std::unordered_map<std::string, UploadSession> upload_sessions_;
    ArtifactStore artifact_store_;
    PeerRegistry peer_registry_;
    DataRouter data_router_;
    ArtifactTransferManager transfer_manager_;
    std::uint64_t data_sequence_{0};
};
