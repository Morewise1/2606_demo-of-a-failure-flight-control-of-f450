#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "../common/protocol.hpp"

namespace amp::linux_core {

class MailboxTransport {
public:
    explicit MailboxTransport(std::string device_path = "/dev/cvi-rtos-cmdqu");
    ~MailboxTransport();

    bool open();
    void close();
    bool isLive() const;

    bool sendSetpoint(const proto::CommandSetpoint& setpoint);
    bool sendArm(proto::ArmState state);
    bool sendHeartbeat(std::uint32_t seq);
    bool sendEmergencyStop(std::uint32_t reason);
    bool sendConfig(const proto::FlightConfig& config);

private:
    enum class SharedRegionBackend {
        Fallback,
        Ion,
        DevMem,
        Shm,
    };

    bool openSharedRegion();
    void closeSharedRegion();
    bool allocateSharedRegion();
    bool allocateIonSharedRegion();
    bool allocateDevmemSharedRegion();
    bool allocateShmSharedRegion();
    bool initializeSharedRegion();
    bool notifySharedRegion();
    bool flushSharedRegion();
    bool sendCommand(std::uint8_t cmd_id, std::uint32_t param, bool wait_ack, std::uint16_t timeout_ms);
    bool writeSetpointCache(const proto::CommandSetpoint& setpoint);
    bool writeArmCache(proto::ArmState state);
    bool writeHeartbeatCache(std::uint32_t seq);
    bool writeEmergencyCache(std::uint32_t reason);
    bool writeConfigCache(const proto::FlightConfig& config);
    std::uint64_t monotonicNowUs() const;

    std::string device_path_;
    std::string devmem_path_{"/dev/mem"};
    std::string shared_region_path_{"/dev/shm/amp_fc_cmd_region.bin"};
    int fd_{-1};
    int ion_fd_{-1};
    int devmem_fd_{-1};
    int shared_fd_{-1};
    proto::SharedCommandRegion* shared_region_{nullptr};
    void* shared_map_base_{nullptr};
    std::size_t shared_map_size_{0};
    proto::SharedCommandRegion fallback_region_{};
    bool fallback_mode_{false};
    SharedRegionBackend shared_backend_{SharedRegionBackend::Fallback};
    std::uintptr_t shared_phys_addr_{0u};
    std::size_t shared_region_size_{sizeof(proto::SharedCommandRegion)};
    std::uint32_t setpoint_seq_{0};
    std::uint32_t arm_seq_{0};
    std::uint32_t heartbeat_seq_{0};
    std::uint32_t emergency_seq_{0};
    std::uint32_t config_seq_{0};
};

}  // namespace amp::linux_core
