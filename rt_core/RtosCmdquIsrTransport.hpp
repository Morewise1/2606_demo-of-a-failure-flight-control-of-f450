#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "../common/protocol.hpp"
#include "FlightRtNode.hpp"

namespace amp::rt {

class RtosCmdquIsrTransport final : public RtTransport {
public:
    explicit RtosCmdquIsrTransport(std::uintptr_t shared_region_addr = 0x83F00000u);
    ~RtosCmdquIsrTransport() override = default;

    bool init() override;
    bool poll(IncomingCommand& command) override;
    bool publish(const proto::RtReport& report) override;

    static int mailboxIsr(int irq, void* context);
    static void onCmdFromMailbox(std::uint8_t cmd_id, std::uint32_t seq_hint);

private:
    struct PendingCmd {
        std::uint8_t cmd_id{0};
        std::uint32_t seq_hint{0};
    };

    bool popCommand(PendingCmd& out);
    bool parseSetpoint(IncomingCommand& command);
    bool parseArm(IncomingCommand& command);
    bool parseHeartbeat(IncomingCommand& command);
    bool parseEmergency(IncomingCommand& command);
    bool parseConfig(IncomingCommand& command);

    static constexpr std::size_t kQueueDepth = 32;
    static std::array<PendingCmd, kQueueDepth> pending_queue_;
    static volatile std::uint32_t queue_head_;
    static volatile std::uint32_t queue_tail_;
    static RtosCmdquIsrTransport* active_instance_;

    proto::SharedCommandRegion* shared_region_{nullptr};
    proto::SharedCommandRegion fallback_region_{};
    std::uintptr_t shared_region_addr_{0};
    std::uint32_t last_setpoint_seq_{0};
    std::uint32_t last_arm_seq_{0};
    std::uint32_t last_heartbeat_seq_{0};
    std::uint32_t last_emergency_seq_{0};
    std::uint32_t last_config_seq_{0};
    std::uint32_t report_seq_{0};
};

}  // namespace amp::rt
