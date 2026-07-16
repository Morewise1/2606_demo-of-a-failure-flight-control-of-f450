#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "../common/protocol.hpp"
#include "Actuator.hpp"
#include "Controller.hpp"
#include "Estimator.hpp"
#include "Receiver.hpp"
#include "Sense.hpp"

namespace amp::rt {

struct IncomingCommand {
    std::optional<proto::CommandSetpoint> setpoint;
    std::optional<proto::CommandArm> arm;
    std::optional<proto::Heartbeat> heartbeat;
    std::optional<proto::FlightConfig> config;
    bool emergency_stop{false};
};

class RtTransport {
public:
    virtual ~RtTransport() = default;
    virtual bool init() = 0;
    virtual bool poll(IncomingCommand& command) = 0;
    virtual bool publish(const proto::RtReport& report) = 0;
};

class LoopbackRtTransport final : public RtTransport {
public:
    bool init() override;
    bool poll(IncomingCommand& command) override;
    bool publish(const proto::RtReport& report) override;

private:
    std::uint64_t tick_{0};
    std::uint32_t publish_count_{0};
};

class FlightRtNode {
public:
    FlightRtNode(
        Sense sense,
        Receiver receiver,
        Estimator estimator,
        Controller controller,
        Actuator actuator,
        std::unique_ptr<RtTransport> transport);
    bool init();
    bool run(std::size_t max_cycles);

private:
    void applyCommand(const IncomingCommand& command, std::uint64_t now_us);
    void applyReceiver(const ReceiverSample& sample, std::uint64_t now_us);
    void enterFailsafe();
    std::uint64_t monotonicNowUs() const;

    Sense sense_;
    Receiver receiver_;
    Estimator estimator_;
    Controller controller_;
    Actuator actuator_;
    std::unique_ptr<RtTransport> transport_;
    std::uint64_t last_setpoint_us_{0};
    std::uint64_t last_heartbeat_us_{0};
    std::uint32_t report_seq_{0};
};

}  // namespace amp::rt
