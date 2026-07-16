#include "TelemetryNode.hpp"

#include "../common/SignalStop.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

namespace amp::linux_core {

namespace {

bool almostEqual(float left, float right, float epsilon = 1e-6f) {
    return std::fabs(left - right) <= epsilon;
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

}  // namespace

bool SimulatedRcSource::init() {
    tick_ = 0;
    return true;
}

RcSnapshot SimulatedRcSource::read() {
    ++tick_;
    RcSnapshot snapshot{};
    snapshot.link_ok = !((tick_ > 140u) && (tick_ < 180u));
    snapshot.setpoint.throttle = 0.33f;
    snapshot.setpoint.roll_deg = 8.0f * std::sin(static_cast<float>(tick_) * 0.03f);
    snapshot.setpoint.pitch_deg = 7.0f * std::sin(static_cast<float>(tick_) * 0.022f);
    snapshot.setpoint.yaw_rate_dps = 25.0f * std::sin(static_cast<float>(tick_) * 0.018f);
    snapshot.setpoint.seq = tick_;
    return snapshot;
}

bool NullRcSource::init() {
    return true;
}

RcSnapshot NullRcSource::read() {
    RcSnapshot snapshot{};
    snapshot.link_ok = false;
    snapshot.setpoint = proto::CommandSetpoint{};
    return snapshot;
}

TelemetryNode::TelemetryNode(std::unique_ptr<RcSource> rc_source, std::string ini_path)
    : mailbox_(), rc_source_(std::move(rc_source)), ini_path_(std::move(ini_path)) {
    if (ini_path_.empty()) {
        if (const char* path = std::getenv("AMP_FLIGHT_CONFIG_INI")) {
            ini_path_ = path;
        }
    }
}

bool TelemetryNode::run(std::size_t max_cycles) {
    if (!rc_source_ || !rc_source_->init()) {
        return false;
    }
    if (!mailbox_.open()) {
        return false;
    }
    if (!pushConfig(active_config_, "boot")) {
        std::cerr << "[Linux] warning: failed to publish boot config\n";
    }

    constexpr std::uint64_t kTickUs = 10000;
    constexpr std::uint64_t kSetpointIntervalUs = 10000;
    constexpr std::uint64_t kHeartbeatIntervalUs = 50000;
    constexpr std::uint64_t kRcTimeoutUs = 200000;
    std::uint64_t last_rc_ok_us = monotonicNowUs();
    bool rc_seen = false;
    bool failsafe_sent = false;
    std::uint64_t setpoint_budget_us = 0;
    std::uint64_t heartbeat_budget_us = 0;
    auto wake = std::chrono::steady_clock::now();

    std::size_t cycle = 0;
    while (((max_cycles == 0u) || (cycle < max_cycles)) && !amp::common::stop_requested()) {
        ++cycle;
        wake += std::chrono::microseconds(kTickUs);
        const std::uint64_t now_us = monotonicNowUs();
        setpoint_budget_us += kTickUs;
        heartbeat_budget_us += kTickUs;

        if (setpoint_budget_us >= kSetpointIntervalUs) {
            setpoint_budget_us = 0;
            RcSnapshot snapshot = rc_source_->read();
            if (snapshot.link_ok) {
                rc_seen = true;
                last_rc_ok_us = now_us;
                failsafe_sent = false;
                mailbox_.sendSetpoint(snapshot.setpoint);
            }
        }

        proto::FlightConfig incoming = active_config_;
        if (maybeReadCliConfig(incoming)) {
            pushConfig(incoming, "cli");
        }
        incoming = active_config_;
        if (maybeLoadIniConfig(incoming)) {
            pushConfig(incoming, "ini");
        }

        if (rc_seen && ((now_us - last_rc_ok_us) > kRcTimeoutUs) && !failsafe_sent) {
            mailbox_.sendEmergencyStop(1u);
            mailbox_.sendArm(proto::ArmState::Failsafe);
            failsafe_sent = true;
            std::cout << "[Linux] rc timeout, failsafe sent\n";
        }

        if (heartbeat_budget_us >= kHeartbeatIntervalUs) {
            heartbeat_budget_us = 0;
            mailbox_.sendHeartbeat(++heartbeat_seq_);
        }
        std::this_thread::sleep_until(wake);
    }

    mailbox_.close();
    return true;
}

std::uint64_t TelemetryNode::monotonicNowUs() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

bool TelemetryNode::parseConfigLine(const std::string& line, proto::FlightConfig& config) const {
    std::string working = line;
    const auto comment = working.find_first_of("#;");
    if (comment != std::string::npos) {
        working = working.substr(0, comment);
    }
    working = trim(working);
    if (working.empty() || working.front() == '[') {
        return false;
    }

    std::string key;
    std::string value;
    if (working.rfind("set ", 0) == 0u) {
        std::istringstream tokens(working.substr(4));
        tokens >> key >> value;
        if (key.empty() || value.empty()) {
            return false;
        }
    } else {
        const auto equals = working.find('=');
        if (equals == std::string::npos) {
            return false;
        }
        key = trim(working.substr(0, equals));
        value = trim(working.substr(equals + 1u));
    }

    key = toLower(trim(key));
    value = trim(value);
    if (key.empty() || value.empty()) {
        return false;
    }

    try {
        if (key == "angle_p" || key == "anglep") {
            config.angle_p = std::stof(value);
            return true;
        }
        if (key == "rate_p" || key == "ratep") {
            config.rate_p = std::stof(value);
            return true;
        }
        if (key == "rate_i" || key == "ratei") {
            config.rate_i = std::stof(value);
            return true;
        }
        if (key == "rate_d" || key == "rated") {
            config.rate_d = std::stof(value);
            return true;
        }
        if (key == "throttle_expo" || key == "throttleexpo" || key == "expo") {
            config.throttle_expo = std::stof(value);
            return true;
        }
        if (key == "deadzone" || key == "deadzone_us") {
            const auto parsed = std::stoi(value);
            config.deadzone_us = static_cast<std::uint16_t>(std::clamp(parsed, 0, 500));
            return true;
        }
    } catch (...) {
        return false;
    }
    return false;
}

bool TelemetryNode::maybeLoadIniConfig(proto::FlightConfig& config) {
    if (ini_path_.empty()) {
        return false;
    }

    std::ifstream file(ini_path_);
    if (!file.is_open()) {
        return false;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();
    if (text == ini_cache_) {
        return false;
    }
    ini_cache_ = text;

    std::istringstream lines(text);
    std::string line;
    bool updated = false;
    while (std::getline(lines, line)) {
        updated = parseConfigLine(line, config) || updated;
    }
    return updated;
}

bool TelemetryNode::maybeReadCliConfig(proto::FlightConfig& config) const {
    if (!std::cin.good() || std::cin.rdbuf() == nullptr) {
        return false;
    }
    if (std::cin.rdbuf()->in_avail() <= 0) {
        return false;
    }

    std::string line;
    if (!std::getline(std::cin, line)) {
        return false;
    }
    line = trim(line);
    if (line.empty()) {
        return false;
    }

    if (toLower(line) == "help") {
        std::cout << "[Linux] CLI: set angle_p|rate_p|rate_i|rate_d|throttle_expo|deadzone <value>\n";
        return false;
    }
    if (toLower(line) == "show") {
        std::cout << "[Linux] active config angle_p=" << active_config_.angle_p << " rate_p=" << active_config_.rate_p
                  << " rate_i=" << active_config_.rate_i << " rate_d=" << active_config_.rate_d
                  << " expo=" << active_config_.throttle_expo << " deadzone=" << active_config_.deadzone_us << '\n';
        return false;
    }
    return parseConfigLine(line, config);
}

bool TelemetryNode::pushConfig(const proto::FlightConfig& config, const char* source) {
    proto::FlightConfig candidate = config;
    candidate.angle_p = std::clamp(candidate.angle_p, 0.5f, 20.0f);
    candidate.rate_p = std::clamp(candidate.rate_p, 0.01f, 1.0f);
    candidate.rate_i = std::clamp(candidate.rate_i, 0.0f, 1.0f);
    candidate.rate_d = std::clamp(candidate.rate_d, 0.0f, 0.1f);
    candidate.throttle_expo = std::clamp(candidate.throttle_expo, 0.0f, 0.95f);
    candidate.deadzone_us = std::clamp<std::uint16_t>(candidate.deadzone_us, 0u, 100u);

    const bool unchanged = almostEqual(candidate.angle_p, active_config_.angle_p) &&
                           almostEqual(candidate.rate_p, active_config_.rate_p) &&
                           almostEqual(candidate.rate_i, active_config_.rate_i) &&
                           almostEqual(candidate.rate_d, active_config_.rate_d) &&
                           almostEqual(candidate.throttle_expo, active_config_.throttle_expo) &&
                           candidate.deadzone_us == active_config_.deadzone_us;
    if (unchanged && config_sent_) {
        return false;
    }

    if (config_sent_) {
        candidate.update_seq = active_config_.update_seq + 1u;
    } else if (candidate.update_seq == 0u) {
        candidate.update_seq = 1u;
    }
    if (!mailbox_.sendConfig(candidate)) {
        return false;
    }
    active_config_ = candidate;
    config_sent_ = true;
    std::cout << "[Linux] config updated from " << source << " seq=" << active_config_.update_seq
              << " angle_p=" << active_config_.angle_p << " rate_p=" << active_config_.rate_p
              << " rate_i=" << active_config_.rate_i << " rate_d=" << active_config_.rate_d
              << " expo=" << active_config_.throttle_expo << " deadzone=" << active_config_.deadzone_us << '\n';
    return true;
}

std::string TelemetryNode::trim(std::string text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1u);
}

}  // namespace amp::linux_core
