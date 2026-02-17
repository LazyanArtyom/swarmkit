#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <type_traits>

#include "swarmkit/core/backend_factory.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::core {
namespace {

constexpr int kDefaultTelemetryRateHz = 5;
constexpr double kInitialLatitudeDeg = 40.1811;
constexpr double kInitialLongitudeDeg = 44.5136;
constexpr float kInitialAltitudeMeters = 10.0F;
constexpr float kInitialBatteryPercent = 95.0F;

class MockBackend final : public IDroneBackend {
   public:
    Result Execute(const CommandContext& context, const Command& command) override {
        (void)context;

        std::visit(
            [](const auto& command_value) {
                using CommandType = std::decay_t<decltype(command_value)>;

                if constexpr (std::is_same_v<CommandType, CmdArm>) {
                    Logger::Info("MockBackend: ARM");
                } else if constexpr (std::is_same_v<CommandType, CmdDisarm>) {
                    Logger::Info("MockBackend: DISARM");
                } else if constexpr (std::is_same_v<CommandType, CmdTakeoff>) {
                    Logger::InfoFmt("MockBackend: TAKEOFF alt_m={}", command_value.alt_m);
                } else if constexpr (std::is_same_v<CommandType, CmdLand>) {
                    Logger::Info("MockBackend: LAND");
                } else if constexpr (std::is_same_v<CommandType, CmdSetRole>) {
                    Logger::InfoFmt("MockBackend: SET_ROLE role={}", command_value.role);
                } else if constexpr (std::is_same_v<CommandType, CmdSendData>) {
                    Logger::InfoFmt("MockBackend: SEND_DATA transfer_id={} dst_agent_id={}",
                                    command_value.ref.transfer_id, command_value.dst_agent_id);
                } else {
                    Logger::Warn("MockBackend: unknown command variant");
                }
            },
            command);

        return Result::Ok();
    }

    void SetTelemetryCallback(TelemetryCallback callback) override {
        telemetry_callback_ = std::move(callback);
    }

    Result StartTelemetry(const std::string& drone_id, int rate_hz) override {
        if (telemetry_running_.exchange(true)) {
            return Result::Rejected("telemetry already running");
        }

        const int effective_rate_hz = (rate_hz <= 0) ? kDefaultTelemetryRateHz : rate_hz;

        telemetry_thread_ = std::thread([this, drone_id, effective_rate_hz]() {
            using namespace std::chrono;

            auto next_tick = steady_clock::now();
            double latitude_deg = kInitialLatitudeDeg;
            double longitude_deg = kInitialLongitudeDeg;
            float relative_alt_m = kInitialAltitudeMeters;
            float battery_percent = kInitialBatteryPercent;

            const auto tick_period = milliseconds(1000 / std::max(1, effective_rate_hz));

            while (telemetry_running_.load()) {
                next_tick += tick_period;

                latitude_deg += 0.00001;
                longitude_deg += 0.00001;
                relative_alt_m += 0.02F;
                battery_percent = std::max(0.0F, battery_percent - 0.01F);

                if (telemetry_callback_) {
                    TelemetryFrame frame;
                    frame.drone_id = drone_id;
                    frame.unix_time_ms =
                        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
                    frame.lat_deg = latitude_deg;
                    frame.lon_deg = longitude_deg;
                    frame.rel_alt_m = relative_alt_m;
                    frame.battery_percent = battery_percent;
                    frame.mode = "SIM";

                    telemetry_callback_(frame);
                }

                std::this_thread::sleep_until(next_tick);
            }
        });

        return Result::Ok();
    }

    Result StopTelemetry(const std::string& drone_id) override {
        (void)drone_id;

        if (!telemetry_running_.exchange(false)) {
            return Result::Ok();
        }

        if (telemetry_thread_.joinable()) {
            telemetry_thread_.join();
        }

        return Result::Ok();
    }

    ~MockBackend() override {
        (void)StopTelemetry("default");
    }

   private:
    TelemetryCallback telemetry_callback_;
    std::atomic<bool> telemetry_running_{false};
    std::thread telemetry_thread_;
};

}  // namespace

DroneBackendPtr MakeMockBackend() {
    return std::make_unique<MockBackend>();
}

}  // namespace swarmkit::core