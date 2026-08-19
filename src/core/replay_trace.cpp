// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/replay_trace.h"

#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace swarmkit::core {

namespace {

std::string EscapeString(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else out += c;
    }
    return out;
}

std::string UnescapeString(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size()) {
            char next = s[++i];
            if (next == '\"') out += '\"';
            else if (next == '\\') out += '\\';
            else if (next == 'n') out += '\n';
            else if (next == 'r') out += '\r';
            else if (next == 't') out += '\t';
            else out += next;
        } else {
            out += s[i];
        }
    }
    return out;
}

}  // namespace

std::string ReplayTrace::ToJsonLines() const {
    std::ostringstream oss;
    oss.imbue(std::locale::classic());
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);

    // Header line
    oss << "{\"type\":\"header\",\"trace_id\":\"" << EscapeString(trace_id)
        << "\",\"version\":\"" << EscapeString(version) << "\"}\n";

    for (const auto& ev : events) {
        std::visit([&oss](const auto& e) {
            using T = std::decay_t<decltype(e)>;
            if constexpr (std::is_same_v<T, EvidenceReceivedEvent>) {
                oss << "{\"type\":\"evidence_received\",\"receive_time_ms\":" << e.receive_time_ms
                    << ",\"agent_id\":\"" << EscapeString(e.agent_id) << "\""
                    << ",\"field_id\":" << static_cast<int>(e.record.identity.field_id)
                    << ",\"sequence\":" << e.record.identity.sequence
                    << ",\"session_id\":\"" << EscapeString(e.record.identity.agent_session_id) << "\""
                    << ",\"source_component\":\"" << EscapeString(e.record.identity.source_component) << "\""
                    << ",\"estimator_id\":\"" << EscapeString(e.record.identity.estimator_id) << "\""
                    << ",\"mission_id\":\"" << EscapeString(e.record.identity.mission_id) << "\""
                    << ",\"mission_revision\":" << e.record.identity.mission_revision
                    << ",\"uncertainty_kind\":" << static_cast<int>(e.record.identity.uncertainty_kind);
                if (e.record.source_time.timestamp_ms.has_value()) {
                    oss << ",\"source_time_ms\":" << *e.record.source_time.timestamp_ms;
                }
                oss << ",\"clock_domain\":" << static_cast<int>(e.record.source_time.clock_domain);
                oss << ",\"clock_sync\":" << static_cast<int>(e.record.source_time.synchronization);
                if (e.record.source_time.clock_uncertainty_ms.has_value()) {
                    oss << ",\"clock_uncertainty_ms\":" << *e.record.source_time.clock_uncertainty_ms;
                }
                oss << ",\"frame\":" << static_cast<int>(e.record.identity.coordinate_frame);
                oss << ",\"estimator_healthy\":" << (e.record.quality.estimator_healthy ? "true" : "false");
                oss << ",\"estimator_position_ok\":" << (e.record.quality.estimator_position_ok ? "true" : "false");
                oss << ",\"estimator_velocity_ok\":" << (e.record.quality.estimator_velocity_ok ? "true" : "false");
                if (e.record.quality.uncertainty.has_value()) {
                    const auto& u = *e.record.quality.uncertainty;
                    oss << ",\"uncertainty\":" << u.value;
                    oss << ",\"uncertainty_semantics\":" << static_cast<int>(u.descriptor.semantics);
                    if (u.descriptor.confidence_level.has_value()) {
                        oss << ",\"uncertainty_conf\":" << *u.descriptor.confidence_level;
                    }
                    if (!u.descriptor.calibration_profile_id.empty()) {
                        oss << ",\"uncertainty_cal_prof\":\"" << EscapeString(u.descriptor.calibration_profile_id) << "\"";
                    }
                    if (!u.descriptor.calibration_version.empty()) {
                        oss << ",\"uncertainty_cal_ver\":\"" << EscapeString(u.descriptor.calibration_version) << "\"";
                    }
                    if (!u.descriptor.source.empty()) {
                        oss << ",\"uncertainty_source\":\"" << EscapeString(u.descriptor.source) << "\"";
                    }
                    if (u.descriptor.measurement_generation != 0) {
                        oss << ",\"uncertainty_meas_gen\":" << u.descriptor.measurement_generation;
                    }
                }
                // Value serialization
                std::visit([&oss](const auto& v) {
                    using VT = std::decay_t<decltype(v)>;
                    if constexpr (std::is_same_v<VT, std::array<double, 3>>) {
                        oss << ",\"val_3d\":[" << v[0] << "," << v[1] << "," << v[2] << "]";
                    } else if constexpr (std::is_same_v<VT, std::array<float, 3>>) {
                        oss << ",\"val_3f\":[" << v[0] << "," << v[1] << "," << v[2] << "]";
                    } else if constexpr (std::is_same_v<VT, float>) {
                        oss << ",\"val_f\":" << v;
                    } else if constexpr (std::is_same_v<VT, bool>) {
                        oss << ",\"val_b\":" << (v ? "true" : "false");
                    } else if constexpr (std::is_same_v<VT, GpsQuality>) {
                        oss << ",\"val_gps\":" << static_cast<int>(v);
                    } else if constexpr (std::is_same_v<VT, EstimatorState>) {
                        oss << ",\"val_est\":" << static_cast<int>(v);
                    }
                }, e.record.value);
                oss << "}\n";
            } else if constexpr (std::is_same_v<T, SessionTransitionEvent>) {
                oss << "{\"type\":\"session_transition\",\"timestamp_ms\":" << e.timestamp_ms
                    << ",\"agent_id\":\"" << EscapeString(e.agent_id) << "\""
                    << ",\"new_session_id\":\"" << EscapeString(e.new_session_id) << "\"}\n";
            } else if constexpr (std::is_same_v<T, ClockModelUpdateEvent>) {
                oss << "{\"type\":\"clock_model_update\",\"timestamp_ms\":" << e.timestamp_ms
                    << ",\"agent_id\":\"" << EscapeString(e.agent_id) << "\""
                    << ",\"offset_estimate_ms\":" << e.clock_state.offset_estimate_ms
                    << ",\"uncertainty_radius_ms\":" << e.clock_state.uncertainty_radius_ms
                    << ",\"max_drift_rate_ppm\":" << e.clock_state.max_drift_rate_ppm
                    << ",\"source_domain\":" << static_cast<int>(e.clock_state.source_domain)
                    << ",\"synchronization\":" << static_cast<int>(e.clock_state.synchronization)
                    << ",\"last_update_ms\":" << e.clock_state.last_update_ms
                    << ",\"deterministic_bound\":" << (e.clock_state.deterministic_bound ? "true" : "false")
                    << ",\"agent_incarnation_id\":\"" << EscapeString(e.clock_state.agent_incarnation_id) << "\""
                    << ",\"clock_model_version\":\"" << EscapeString(e.clock_state.clock_model_version) << "\"}\n";
            } else if constexpr (std::is_same_v<T, MembershipChangeEvent>) {
                oss << "{\"type\":\"membership_change\",\"timestamp_ms\":" << e.timestamp_ms
                    << ",\"membership_revision\":" << e.participants.membership_revision
                    << ",\"agents\":[";
                for (std::size_t i = 0; i < e.participants.agent_ids.size(); ++i) {
                    if (i > 0) oss << ",";
                    oss << "\"" << EscapeString(e.participants.agent_ids[i]) << "\"";
                }
                oss << "]}\n";
            } else if constexpr (std::is_same_v<T, SnapshotRequestEvent>) {
                oss << "{\"type\":\"snapshot_request\",\"request_id\":\"" << EscapeString(e.request_id) << "\""
                    << ",\"evaluation_time_ms\":" << e.evaluation_time_ms
                    << ",\"evidence_freeze_ms\":" << e.evidence_freeze_ms
                    << ",\"contract_hash\":\"" << EscapeString(e.contract_hash) << "\""
                    << ",\"membership_revision\":" << e.participants.membership_revision
                    << ",\"participants\":[";
                for (std::size_t i = 0; i < e.participants.agent_ids.size(); ++i) {
                    if (i > 0) oss << ",";
                    oss << "\"" << EscapeString(e.participants.agent_ids[i]) << "\"";
                }
                oss << "]";
                if (e.certificate.has_value()) {
                    oss << ",\"certificate_serialized\":\"" << EscapeString(SerializeCertificate(*e.certificate)) << "\"";
                }
                oss << "}\n";
            }
        }, ev);
    }
    return oss.str();
}

std::optional<ReplayTrace> ReplayTrace::FromJsonLines(std::string_view json_lines) {
    ReplayTrace trace;
    std::istringstream iss{std::string(json_lines)};
    std::string line;

    while (std::getline(iss, line)) {
        if (line.empty()) continue;

        if (line.find("\"type\":\"header\"") != std::string::npos) {
            auto pos = line.find("\"trace_id\":\"");
            if (pos != std::string::npos) {
                auto end_pos = line.find('"', pos + 12);
                if (end_pos != std::string::npos) {
                    trace.trace_id = line.substr(pos + 12, end_pos - (pos + 12));
                }
            }
            auto ver_pos = line.find("\"version\":\"");
            if (ver_pos != std::string::npos) {
                auto end_pos = line.find('"', ver_pos + 11);
                if (end_pos != std::string::npos) {
                    trace.version = line.substr(ver_pos + 11, end_pos - (ver_pos + 11));
                }
            }
        } else if (line.find("\"type\":\"session_transition\"") != std::string::npos) {
            SessionTransitionEvent ev;
            auto ts_pos = line.find("\"timestamp_ms\":");
            if (ts_pos != std::string::npos) {
                ev.timestamp_ms = std::stoll(line.substr(ts_pos + 15));
            }
            auto id_pos = line.find("\"agent_id\":\"");
            if (id_pos != std::string::npos) {
                auto end_pos = line.find('"', id_pos + 12);
                if (end_pos != std::string::npos) {
                    ev.agent_id = line.substr(id_pos + 12, end_pos - (id_pos + 12));
                }
            }
            auto sess_pos = line.find("\"new_session_id\":\"");
            if (sess_pos != std::string::npos) {
                auto end_pos = line.find('"', sess_pos + 18);
                if (end_pos != std::string::npos) {
                    ev.new_session_id = line.substr(sess_pos + 18, end_pos - (sess_pos + 18));
                }
            }
            trace.events.push_back(ev);
        } else if (line.find("\"type\":\"evidence_received\"") != std::string::npos) {
            EvidenceReceivedEvent ev;
            auto rx_pos = line.find("\"receive_time_ms\":");
            if (rx_pos != std::string::npos) {
                ev.receive_time_ms = std::stoll(line.substr(rx_pos + 18));
                ev.record.receive_time_ms = ev.receive_time_ms;
            }
            auto id_pos = line.find("\"agent_id\":\"");
            if (id_pos != std::string::npos) {
                auto end_pos = line.find('"', id_pos + 12);
                if (end_pos != std::string::npos) {
                    ev.agent_id = line.substr(id_pos + 12, end_pos - (id_pos + 12));
                    ev.record.identity.agent_id = ev.agent_id;
                }
            }
            auto f_pos = line.find("\"field_id\":");
            if (f_pos != std::string::npos) {
                ev.record.identity.field_id = static_cast<EvidenceFieldId>(std::stoi(line.substr(f_pos + 11)));
            }
            auto seq_pos = line.find("\"sequence\":");
            if (seq_pos != std::string::npos) {
                ev.record.identity.sequence = std::stoull(line.substr(seq_pos + 11));
            }
            auto sess_pos = line.find("\"session_id\":\"");
            if (sess_pos != std::string::npos) {
                auto end_pos = line.find('"', sess_pos + 14);
                if (end_pos != std::string::npos) {
                    ev.record.identity.agent_session_id = line.substr(sess_pos + 14, end_pos - (sess_pos + 14));
                }
            }
            auto src_pos = line.find("\"source_component\":\"");
            if (src_pos != std::string::npos) {
                auto end_pos = line.find('"', src_pos + 20);
                if (end_pos != std::string::npos) {
                    ev.record.identity.source_component = line.substr(src_pos + 20, end_pos - (src_pos + 20));
                }
            }
            auto estid_pos = line.find("\"estimator_id\":\"");
            if (estid_pos != std::string::npos) {
                auto end_pos = line.find('"', estid_pos + 16);
                if (end_pos != std::string::npos) {
                    ev.record.identity.estimator_id = line.substr(estid_pos + 16, end_pos - (estid_pos + 16));
                }
            }
            auto misid_pos = line.find("\"mission_id\":\"");
            if (misid_pos != std::string::npos) {
                auto end_pos = line.find('"', misid_pos + 14);
                if (end_pos != std::string::npos) {
                    ev.record.identity.mission_id = line.substr(misid_pos + 14, end_pos - (misid_pos + 14));
                }
            }
            auto misrev_pos = line.find("\"mission_revision\":");
            if (misrev_pos != std::string::npos) {
                ev.record.identity.mission_revision = std::stoull(line.substr(misrev_pos + 19));
            }
            auto unckind_pos = line.find("\"uncertainty_kind\":");
            if (unckind_pos != std::string::npos) {
                ev.record.identity.uncertainty_kind = static_cast<UncertaintySemantics>(std::stoi(line.substr(unckind_pos + 19)));
            }
            auto st_pos = line.find("\"source_time_ms\":");
            if (st_pos != std::string::npos) {
                ev.record.source_time.timestamp_ms = std::stoll(line.substr(st_pos + 17));
            }
            auto dom_pos = line.find("\"clock_domain\":");
            if (dom_pos != std::string::npos) {
                ev.record.source_time.clock_domain = static_cast<ClockDomain>(std::stoi(line.substr(dom_pos + 15)));
            }
            auto sync_pos = line.find("\"clock_sync\":");
            if (sync_pos != std::string::npos) {
                ev.record.source_time.synchronization = static_cast<ClockSynchronization>(std::stoi(line.substr(sync_pos + 13)));
            }
            auto cu_pos = line.find("\"clock_uncertainty_ms\":");
            if (cu_pos != std::string::npos) {
                ev.record.source_time.clock_uncertainty_ms = std::stod(line.substr(cu_pos + 23));
            }
            auto fr_pos = line.find("\"frame\":");
            if (fr_pos != std::string::npos) {
                ev.record.identity.coordinate_frame = static_cast<CoordinateFrame>(std::stoi(line.substr(fr_pos + 8)));
            }
            ev.record.quality.estimator_healthy = (line.find("\"estimator_healthy\":true") != std::string::npos);
            ev.record.quality.estimator_position_ok = (line.find("\"estimator_position_ok\":true") != std::string::npos);
            ev.record.quality.estimator_velocity_ok = (line.find("\"estimator_velocity_ok\":true") != std::string::npos);

            auto unc_pos = line.find("\"uncertainty\":");
            if (unc_pos != std::string::npos) {
                UncertaintyEstimate unc;
                unc.value = std::stof(line.substr(unc_pos + 14));
                auto sem_pos = line.find("\"uncertainty_semantics\":");
                if (sem_pos != std::string::npos) {
                    unc.descriptor.semantics = static_cast<UncertaintySemantics>(std::stoi(line.substr(sem_pos + 24)));
                }
                auto conf_pos = line.find("\"uncertainty_conf\":");
                if (conf_pos != std::string::npos) {
                    unc.descriptor.confidence_level = std::stod(line.substr(conf_pos + 19));
                }
                auto calprof_pos = line.find("\"uncertainty_cal_prof\":\"");
                if (calprof_pos != std::string::npos) {
                    auto end_pos = line.find('"', calprof_pos + 24);
                    if (end_pos != std::string::npos) {
                        unc.descriptor.calibration_profile_id = line.substr(calprof_pos + 24, end_pos - (calprof_pos + 24));
                    }
                }
                auto calver_pos = line.find("\"uncertainty_cal_ver\":\"");
                if (calver_pos != std::string::npos) {
                    auto end_pos = line.find('"', calver_pos + 23);
                    if (end_pos != std::string::npos) {
                        unc.descriptor.calibration_version = line.substr(calver_pos + 23, end_pos - (calver_pos + 23));
                    }
                }
                auto src_pos2 = line.find("\"uncertainty_source\":\"");
                if (src_pos2 != std::string::npos) {
                    auto end_pos = line.find('"', src_pos2 + 22);
                    if (end_pos != std::string::npos) {
                        unc.descriptor.source = line.substr(src_pos2 + 22, end_pos - (src_pos2 + 22));
                    }
                }
                auto measgen_pos = line.find("\"uncertainty_meas_gen\":");
                if (measgen_pos != std::string::npos) {
                    unc.descriptor.measurement_generation = std::stoull(line.substr(measgen_pos + 23));
                }
                ev.record.quality.uncertainty = unc;
            }

            auto val3d_pos = line.find("\"val_3d\":[");
            if (val3d_pos != std::string::npos) {
                std::array<double, 3> pos_arr{};
                std::sscanf(line.c_str() + val3d_pos + 10, "%lf,%lf,%lf", &pos_arr[0], &pos_arr[1], &pos_arr[2]);
                ev.record.value = pos_arr;
            }
            auto val3f_pos = line.find("\"val_3f\":[");
            if (val3f_pos != std::string::npos) {
                std::array<float, 3> vel_arr{};
                std::sscanf(line.c_str() + val3f_pos + 10, "%f,%f,%f", &vel_arr[0], &vel_arr[1], &vel_arr[2]);
                ev.record.value = vel_arr;
            }
            auto valf_pos = line.find("\"val_f\":");
            if (valf_pos != std::string::npos) {
                ev.record.value = std::stof(line.substr(valf_pos + 8));
            }
            auto valb_pos = line.find("\"val_b\":");
            if (valb_pos != std::string::npos) {
                ev.record.value = (line.substr(valb_pos + 8, 4) == "true");
            }
            auto valest_pos = line.find("\"val_est\":");
            if (valest_pos != std::string::npos) {
                ev.record.value = static_cast<EstimatorState>(std::stoi(line.substr(valest_pos + 10)));
            }
            auto valgps_pos = line.find("\"val_gps\":");
            if (valgps_pos != std::string::npos) {
                ev.record.value = static_cast<GpsQuality>(std::stoi(line.substr(valgps_pos + 10)));
            }

            trace.events.push_back(ev);
        } else if (line.find("\"type\":\"clock_model_update\"") != std::string::npos) {
            ClockModelUpdateEvent ev;
            auto ts_pos = line.find("\"timestamp_ms\":");
            if (ts_pos != std::string::npos) {
                ev.timestamp_ms = std::stoll(line.substr(ts_pos + 15));
            }
            auto id_pos = line.find("\"agent_id\":\"");
            if (id_pos != std::string::npos) {
                auto end_pos = line.find('"', id_pos + 12);
                if (end_pos != std::string::npos) {
                    ev.agent_id = line.substr(id_pos + 12, end_pos - (id_pos + 12));
                }
            }
            auto off_pos = line.find("\"offset_estimate_ms\":");
            if (off_pos != std::string::npos) {
                ev.clock_state.offset_estimate_ms = std::stod(line.substr(off_pos + 21));
            }
            auto unc_pos = line.find("\"uncertainty_radius_ms\":");
            if (unc_pos != std::string::npos) {
                ev.clock_state.uncertainty_radius_ms = std::stod(line.substr(unc_pos + 24));
            }
            auto dr_pos = line.find("\"max_drift_rate_ppm\":");
            if (dr_pos != std::string::npos) {
                ev.clock_state.max_drift_rate_ppm = std::stod(line.substr(dr_pos + 21));
            }
            auto dom_pos = line.find("\"source_domain\":");
            if (dom_pos != std::string::npos) {
                ev.clock_state.source_domain = static_cast<ClockDomain>(std::stoi(line.substr(dom_pos + 16)));
            }
            auto syn_pos = line.find("\"synchronization\":");
            if (syn_pos != std::string::npos) {
                ev.clock_state.synchronization = static_cast<ClockSynchronization>(std::stoi(line.substr(syn_pos + 18)));
            }
            auto lu_pos = line.find("\"last_update_ms\":");
            if (lu_pos != std::string::npos) {
                ev.clock_state.last_update_ms = std::stoll(line.substr(lu_pos + 17));
            }
            ev.clock_state.deterministic_bound = (line.find("\"deterministic_bound\":true") != std::string::npos);
            auto inc_pos = line.find("\"agent_incarnation_id\":\"");
            if (inc_pos != std::string::npos) {
                auto end_pos = line.find('"', inc_pos + 24);
                if (end_pos != std::string::npos) {
                    ev.clock_state.agent_incarnation_id = line.substr(inc_pos + 24, end_pos - (inc_pos + 24));
                }
            }
            auto cmv_pos = line.find("\"clock_model_version\":\"");
            if (cmv_pos != std::string::npos) {
                auto end_pos = line.find('"', cmv_pos + 23);
                if (end_pos != std::string::npos) {
                    ev.clock_state.clock_model_version = line.substr(cmv_pos + 23, end_pos - (cmv_pos + 23));
                }
            }
            trace.events.push_back(ev);
        } else if (line.find("\"type\":\"membership_change\"") != std::string::npos) {
            MembershipChangeEvent ev;
            auto ts_pos = line.find("\"timestamp_ms\":");
            if (ts_pos != std::string::npos) {
                ev.timestamp_ms = std::stoll(line.substr(ts_pos + 15));
            }
            auto rev_pos = line.find("\"membership_revision\":");
            if (rev_pos != std::string::npos) {
                ev.participants.membership_revision = std::stoull(line.substr(rev_pos + 22));
            }
            auto agents_pos = line.find("\"agents\":[");
            if (agents_pos != std::string::npos) {
                auto end_agents = line.find(']', agents_pos + 10);
                if (end_agents != std::string::npos) {
                    std::string agents_str = line.substr(agents_pos + 10, end_agents - (agents_pos + 10));
                    std::istringstream aiss{agents_str};
                    std::string a_token;
                    while (std::getline(aiss, a_token, ',')) {
                        while (!a_token.empty() && (a_token.front() == ' ' || a_token.front() == '"')) a_token.erase(0, 1);
                        while (!a_token.empty() && (a_token.back() == ' ' || a_token.back() == '"')) a_token.pop_back();
                        if (!a_token.empty()) {
                            ev.participants.agent_ids.push_back(a_token);
                        }
                    }
                }
            }
            trace.events.push_back(ev);
        } else if (line.find("\"type\":\"snapshot_request\"") != std::string::npos) {
            SnapshotRequestEvent ev;
            auto req_pos = line.find("\"request_id\":\"");
            if (req_pos != std::string::npos) {
                auto end_pos = line.find('"', req_pos + 14);
                if (end_pos != std::string::npos) {
                    ev.request_id = line.substr(req_pos + 14, end_pos - (req_pos + 14));
                }
            }
            auto t_pos = line.find("\"evaluation_time_ms\":");
            if (t_pos != std::string::npos) {
                ev.evaluation_time_ms = std::stod(line.substr(t_pos + 21));
            }
            auto r_pos = line.find("\"evidence_freeze_ms\":");
            if (r_pos != std::string::npos) {
                ev.evidence_freeze_ms = std::stoll(line.substr(r_pos + 21));
            }
            auto ch_pos = line.find("\"contract_hash\":\"");
            if (ch_pos != std::string::npos) {
                auto end_pos = line.find('"', ch_pos + 17);
                if (end_pos != std::string::npos) {
                    ev.contract_hash = line.substr(ch_pos + 17, end_pos - (ch_pos + 17));
                }
            }
            auto rev_pos = line.find("\"membership_revision\":");
            if (rev_pos != std::string::npos) {
                ev.participants.membership_revision = std::stoull(line.substr(rev_pos + 22));
            }
            auto part_pos = line.find("\"participants\":[");
            if (part_pos != std::string::npos) {
                auto end_part = line.find(']', part_pos + 16);
                if (end_part != std::string::npos) {
                    std::string part_str = line.substr(part_pos + 16, end_part - (part_pos + 16));
                    std::istringstream piss{part_str};
                    std::string p_token;
                    while (std::getline(piss, p_token, ',')) {
                        while (!p_token.empty() && (p_token.front() == ' ' || p_token.front() == '"')) p_token.erase(0, 1);
                        while (!p_token.empty() && (p_token.back() == ' ' || p_token.back() == '"')) p_token.pop_back();
                        if (!p_token.empty()) {
                            ev.participants.agent_ids.push_back(p_token);
                        }
                    }
                }
            }
            auto cert_pos = line.find("\"certificate_serialized\":\"");
            if (cert_pos != std::string::npos) {
                auto end_pos = line.find('"', cert_pos + 26);
                if (end_pos != std::string::npos) {
                    std::string cert_str = line.substr(cert_pos + 26, end_pos - (cert_pos + 26));
                    ev.certificate = DeserializeCertificate(UnescapeString(cert_str));
                }
            }
            trace.events.push_back(ev);
        }
    }

    return trace;
}

bool ReplayTrace::SaveToFile(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << ToJsonLines();
    return ofs.good();
}

std::optional<ReplayTrace> ReplayTrace::LoadFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return std::nullopt;
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
    return FromJsonLines(content);
}

}  // namespace swarmkit::core
