#include "swarmkit/agent/sim_backend.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"

namespace swarmkit::agent {
namespace {

constexpr int    kDefaultRateHz      = 5;
constexpr double kInitialLatDeg      = 40.1811;
constexpr double kInitialLonDeg      = 44.5136;
constexpr float  kInitialAltMeters   = 10.0F;
constexpr float  kInitialBatteryPct  = 95.0F;
constexpr double kGpsDriftDegPerTick = 0.00001;
constexpr float  kAltClimbMPerTick   = 0.02F;
constexpr float  kBatteryDrainPct    = 0.01F;

// ---------------------------------------------------------------------------
// SimBackend — built-in drone simulator for development and testing.
//
// Accepts every command (logs it and returns Ok).
// Generates synthetic telemetry at the requested rate: GPS drifts slightly,
// altitude climbs, battery drains — just enough to verify the pipeline.
// ---------------------------------------------------------------------------
class SimBackend final : public IDroneBackend {
   public:
    ~SimBackend() override {
        (void)StopTelemetry("default");
    }

    core::Result Execute(const CommandEnvelope& envelope) override {
        const CommandContext& ctx = envelope.context;

        std::visit(core::Overloaded{

            // ---- Flight ----------------------------------------------------
            [&](const FlightCmd& cmd) {
                std::visit(core::Overloaded{
                    [&](const CmdArm&) {
                        core::Logger::InfoFmt("SimBackend: ARM  drone={}",
                                              ctx.drone_id);
                    },
                    [&](const CmdDisarm&) {
                        core::Logger::InfoFmt("SimBackend: DISARM  drone={}",
                                              ctx.drone_id);
                    },
                    [&](const CmdTakeoff& cmd) {
                        core::Logger::InfoFmt("SimBackend: TAKEOFF alt_m={}  drone={}",
                                              cmd.alt_m, ctx.drone_id);
                    },
                    [&](const CmdLand&) {
                        core::Logger::InfoFmt("SimBackend: LAND  drone={}",
                                              ctx.drone_id);
                    },
                }, cmd);
            },

            // ---- Navigation ------------------------------------------------
            [&](const NavCmd& cmd) {
                std::visit(core::Overloaded{
                    [&](const CmdSetWaypoint& cmd) {
                        core::Logger::InfoFmt(
                            "SimBackend: WAYPOINT lat={} lon={} alt={}m  drone={}",
                            cmd.lat_deg, cmd.lon_deg, cmd.alt_m, ctx.drone_id);
                    },
                    [&](const CmdReturnHome&) {
                        core::Logger::InfoFmt("SimBackend: RETURN_HOME  drone={}",
                                              ctx.drone_id);
                    },
                    [&](const CmdHoldPosition&) {
                        core::Logger::InfoFmt("SimBackend: HOLD  drone={}",
                                              ctx.drone_id);
                    },
                }, cmd);
            },

            // ---- Swarm -----------------------------------------------------
            [&](const SwarmCmd& cmd) {
                std::visit(core::Overloaded{
                    [&](const CmdSetRole& cmd) {
                        core::Logger::InfoFmt(
                            "SimBackend: SET_ROLE role={}  drone={}",
                            cmd.role, ctx.drone_id);
                    },
                    [&](const CmdSetFormation& cmd) {
                        core::Logger::InfoFmt(
                            "SimBackend: SET_FORMATION id={} slot={}  drone={}",
                            cmd.formation_id, cmd.slot_index, ctx.drone_id);
                    },
                    [&](const CmdRunSequence& cmd) {
                        core::Logger::InfoFmt(
                            "SimBackend: RUN_SEQUENCE id={} sync_ms={}  drone={}",
                            cmd.sequence_id, cmd.sync_unix_ms, ctx.drone_id);
                    },
                }, cmd);
            },

            // ---- Payload ---------------------------------------------------
            [&](const PayloadCmd&) {
                core::Logger::WarnFmt(
                    "SimBackend: payload commands not implemented  drone={}",
                    ctx.drone_id);
            },

        }, envelope.command);

        return core::Result::Ok();
    }

    core::Result StartTelemetry(const std::string& drone_id,
                                int                rate_hz,
                                TelemetryCallback  callback) override {
        if (running_.exchange(true)) {
            return core::Result::Rejected("telemetry already running");
        }

        const int kEffectiveRate = (rate_hz <= 0) ? kDefaultRateHz : rate_hz;

        thread_ = std::thread(
            [this, drone_id, kEffectiveRate, callback = std::move(callback)]() {
                using std::chrono::milliseconds;
                using std::chrono::steady_clock;
                using std::chrono::system_clock;
                using std::chrono::duration_cast;

                auto   next_tick   = steady_clock::now();
                double lat_deg     = kInitialLatDeg;
                double lon_deg     = kInitialLonDeg;
                float  alt_m       = kInitialAltMeters;
                float  battery_pct = kInitialBatteryPct;

                const auto kPeriod = milliseconds(1000 / std::max(1, kEffectiveRate));

                while (running_.load()) {
                    next_tick += kPeriod;

                    lat_deg     += kGpsDriftDegPerTick;
                    lon_deg     += kGpsDriftDegPerTick;
                    alt_m       += kAltClimbMPerTick;
                    battery_pct  = std::max(0.0F, battery_pct - kBatteryDrainPct);

                    core::TelemetryFrame frame;
                    frame.drone_id        = drone_id;
                    frame.unix_time_ms    = duration_cast<milliseconds>(
                        system_clock::now().time_since_epoch())
                                                .count();
                    frame.lat_deg         = lat_deg;
                    frame.lon_deg         = lon_deg;
                    frame.rel_alt_m       = alt_m;
                    frame.battery_percent = battery_pct;
                    frame.mode            = "SIM";

                    callback(frame);

                    std::this_thread::sleep_until(next_tick);
                }
            });

        return core::Result::Ok();
    }

    core::Result StopTelemetry(const std::string& /*drone_id*/) override {
        if (!running_.exchange(false)) {
            return core::Result::Ok();
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        return core::Result::Ok();
    }

   private:
    std::atomic<bool> running_{false};
    std::thread       thread_;
};

}  // namespace

DroneBackendPtr MakeSimBackend() {
    return std::make_unique<SimBackend>();
}

}  // namespace swarmkit::agent
