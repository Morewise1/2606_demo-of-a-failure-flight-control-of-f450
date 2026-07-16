#include "RtosCmdquIsrTransport.hpp"

#include <atomic>
#include <cstring>

namespace amp::rt {

std::array<RtosCmdquIsrTransport::PendingCmd, RtosCmdquIsrTransport::kQueueDepth> RtosCmdquIsrTransport::pending_queue_{};
volatile std::uint32_t RtosCmdquIsrTransport::queue_head_ = 0;
volatile std::uint32_t RtosCmdquIsrTransport::queue_tail_ = 0;
RtosCmdquIsrTransport* RtosCmdquIsrTransport::active_instance_ = nullptr;

RtosCmdquIsrTransport::RtosCmdquIsrTransport(std::uintptr_t shared_region_addr) : shared_region_addr_(shared_region_addr) {}

bool RtosCmdquIsrTransport::init() {
    std::memset(&fallback_region_, 0, sizeof(fallback_region_));
    fallback_region_.magic = proto::kSharedRegionMagic;
    fallback_region_.version = proto::kSharedRegionVersion;

#if defined(AMP_RTOS_FREERTOS)
    shared_region_ = reinterpret_cast<proto::SharedCommandRegion*>(shared_region_addr_);
#else
    shared_region_ = &fallback_region_;
#endif
    if (shared_region_ == nullptr) {
        return false;
    }

    if (shared_region_->magic != proto::kSharedRegionMagic || shared_region_->version != proto::kSharedRegionVersion) {
        std::memset(shared_region_, 0, sizeof(*shared_region_));
        shared_region_->magic = proto::kSharedRegionMagic;
        shared_region_->version = proto::kSharedRegionVersion;
    }

    last_setpoint_seq_ = 0;
    last_arm_seq_ = 0;
    last_heartbeat_seq_ = 0;
    last_emergency_seq_ = 0;
    last_config_seq_ = 0;
    report_seq_ = 0;
    queue_head_ = 0;
    queue_tail_ = 0;
    active_instance_ = this;
    return true;
}

bool RtosCmdquIsrTransport::poll(IncomingCommand& command) {
    command = IncomingCommand{};
    if (parseEmergency(command)) {
        return true;
    }

    PendingCmd pending{};
    while (popCommand(pending)) {
        switch (static_cast<proto::FlightCmdId>(pending.cmd_id)) {
            case proto::FlightCmdId::EmergencyStop:
                if (parseEmergency(command)) {
                    return true;
                }
                break;
            case proto::FlightCmdId::Arm:
                parseArm(command);
                break;
            case proto::FlightCmdId::Setpoint:
                parseSetpoint(command);
                break;
            case proto::FlightCmdId::Heartbeat:
                parseHeartbeat(command);
                break;
            case proto::FlightCmdId::Config:
                parseConfig(command);
                break;
            default:
                break;
        }
    }

    parseArm(command);
    parseSetpoint(command);
    parseHeartbeat(command);
    parseConfig(command);
    return true;
}

bool RtosCmdquIsrTransport::publish(const proto::RtReport& report) {
    if (shared_region_ == nullptr) {
        return false;
    }
    ++report_seq_;
    proto::RtReport payload = report;
    payload.seq = report.seq == 0u ? report_seq_ : report.seq;
    shared_region_->rt_report.payload = payload;
    shared_region_->rt_report.payload_crc = proto::payload_crc(payload);
    std::atomic_thread_fence(std::memory_order_release);
    shared_region_->rt_report.seq = payload.seq;
    std::atomic_thread_fence(std::memory_order_release);
    return true;
}

int RtosCmdquIsrTransport::mailboxIsr(int irq, void* context) {
    (void)irq;
    (void)context;
    return 0;
}

void RtosCmdquIsrTransport::onCmdFromMailbox(std::uint8_t cmd_id, std::uint32_t seq_hint) {
    RtosCmdquIsrTransport* instance = active_instance_;
    if (instance == nullptr) {
        return;
    }
    const std::uint32_t head = queue_head_;
    const std::uint32_t next = (head + 1u) % static_cast<std::uint32_t>(kQueueDepth);
    if (next == queue_tail_) {
        return;
    }
    pending_queue_[head].cmd_id = cmd_id;
    pending_queue_[head].seq_hint = seq_hint;
    std::atomic_thread_fence(std::memory_order_release);
    queue_head_ = next;
}

bool RtosCmdquIsrTransport::popCommand(PendingCmd& out) {
    const std::uint32_t tail = queue_tail_;
    if (tail == queue_head_) {
        return false;
    }
    out = pending_queue_[tail];
    std::atomic_thread_fence(std::memory_order_acquire);
    queue_tail_ = (tail + 1u) % static_cast<std::uint32_t>(kQueueDepth);
    return true;
}

bool RtosCmdquIsrTransport::parseSetpoint(IncomingCommand& command) {
    if (shared_region_ == nullptr) {
        return false;
    }
    const auto& slot = shared_region_->setpoint;
    if (!proto::verify_slot(slot) || slot.seq <= last_setpoint_seq_) {
        return false;
    }
    last_setpoint_seq_ = slot.seq;
    command.setpoint = slot.payload;
    return true;
}

bool RtosCmdquIsrTransport::parseArm(IncomingCommand& command) {
    if (shared_region_ == nullptr) {
        return false;
    }
    const auto& slot = shared_region_->arm;
    if (!proto::verify_slot(slot) || slot.seq <= last_arm_seq_) {
        return false;
    }
    last_arm_seq_ = slot.seq;
    command.arm = slot.payload;
    return true;
}

bool RtosCmdquIsrTransport::parseHeartbeat(IncomingCommand& command) {
    if (shared_region_ == nullptr) {
        return false;
    }
    const auto& slot = shared_region_->heartbeat;
    if (!proto::verify_slot(slot) || slot.seq <= last_heartbeat_seq_) {
        return false;
    }
    last_heartbeat_seq_ = slot.seq;
    command.heartbeat = slot.payload;
    return true;
}

bool RtosCmdquIsrTransport::parseEmergency(IncomingCommand& command) {
    if (shared_region_ == nullptr) {
        return false;
    }
    const auto& slot = shared_region_->emergency_stop;
    if (!proto::verify_slot(slot) || slot.seq <= last_emergency_seq_) {
        return false;
    }
    last_emergency_seq_ = slot.seq;
    command.emergency_stop = true;
    return true;
}

bool RtosCmdquIsrTransport::parseConfig(IncomingCommand& command) {
    if (shared_region_ == nullptr) {
        return false;
    }
    const auto& slot = shared_region_->config;
    if (!proto::verify_slot(slot) || slot.seq <= last_config_seq_) {
        return false;
    }
    last_config_seq_ = slot.seq;
    command.config = slot.payload;
    return true;
}

}  // namespace amp::rt
