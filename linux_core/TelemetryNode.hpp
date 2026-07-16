#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "../common/protocol.hpp"
#include "MailboxTransport.hpp"

namespace amp::linux_core {

struct RcSnapshot {
    bool link_ok;
    proto::CommandSetpoint setpoint;
};

class RcSource {
public:
    virtual ~RcSource() = default;
    virtual bool init() = 0;
    virtual RcSnapshot read() = 0;
};

class SimulatedRcSource final : public RcSource {
public:
    bool init() override;
    RcSnapshot read() override;

private:
    std::uint32_t tick_{0};
};

class NullRcSource final : public RcSource {
public:
    bool init() override;
    RcSnapshot read() override;
};

class TelemetryNode {
public:
    explicit TelemetryNode(std::unique_ptr<RcSource> rc_source, std::string ini_path = "");
    bool run(std::size_t max_cycles);

private:
    bool parseConfigLine(const std::string& line, proto::FlightConfig& config) const;
    bool maybeLoadIniConfig(proto::FlightConfig& config);
    bool maybeReadCliConfig(proto::FlightConfig& config) const;
    bool pushConfig(const proto::FlightConfig& config, const char* source);
    static std::string trim(std::string text);
    std::uint64_t monotonicNowUs() const;

    MailboxTransport mailbox_;
    std::unique_ptr<RcSource> rc_source_;
    std::uint32_t heartbeat_seq_{0};
    proto::FlightConfig active_config_{proto::default_flight_config()};
    bool config_sent_{false};
    std::string ini_path_;
    std::string ini_cache_;
};

}  // namespace amp::linux_core
