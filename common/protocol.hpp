#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace amp::proto {

constexpr std::uint32_t kMagic = 0x414D5031u;
constexpr std::uint8_t kVersion = 1;
constexpr std::size_t kMotorCount = 4;
constexpr std::uint16_t kPwmMinUs = 1000;
constexpr std::uint16_t kPwmMaxUs = 2000;
constexpr std::uint32_t kSharedRegionMagic = 0x4643414Du;
constexpr std::uint32_t kSharedRegionVersion = 1u;

// Mailbox is used only as a low-rate doorbell/control plane:
// InitSharedMemory publishes the shared region physical address once, then
// Config/Arm/EmergencyStop notify RTOS that a shared-memory slot changed.
// Setpoint and Heartbeat are high-rate data-plane slots and do not use mailbox.
enum class FlightCmdId : std::uint8_t {
    InitSharedMemory = 0x20,
    Setpoint = 0x30,
    Arm = 0x31,
    Heartbeat = 0x32,
    EmergencyStop = 0x33,
    Config = 0x34,
};

enum class MessageKind : std::uint8_t {
    Setpoint = 1,
    Arm = 2,
    RtReport = 3,
    Heartbeat = 4,
    EmergencyStop = 5
};

enum class ArmState : std::uint8_t {
    Disarmed = 0,
    Armed = 1,
    Failsafe = 2
};

enum StatusFlag : std::uint32_t {
    StatusArmed = 1u << 0u,
    StatusFailsafe = 1u << 1u,
    StatusImuHealthy = 1u << 2u,
    StatusRcHealthy = 1u << 3u
};

#pragma pack(push, 1)
struct FrameHeader {
    std::uint32_t magic;
    std::uint16_t payload_size;
    std::uint8_t message_kind;
    std::uint8_t version;
    std::uint32_t crc32;
};

struct CommandSetpoint {
    float throttle;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    std::uint32_t seq;
};

struct CommandArm {
    std::uint8_t state;
    std::uint8_t reserved0;
    std::uint8_t reserved1;
    std::uint8_t reserved2;
};

struct CommandEmergencyStop {
    std::uint32_t reason;
};

struct Heartbeat {
    std::uint64_t monotonic_us;
    std::uint32_t seq;
};

struct FlightConfig {
    float angle_p;
    float rate_p;
    float rate_i;
    float rate_d;
    float throttle_expo;
    std::uint16_t deadzone_us;
    std::uint16_t reserved0;
    std::uint32_t update_seq;
};

struct RtReport {
    std::uint64_t monotonic_us;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    std::array<std::uint16_t, kMotorCount> motor_pwm_us;
    std::uint32_t status_flags;
    std::uint32_t seq;
};

template <typename T>
struct SharedSlot {
    std::uint32_t seq;
    T payload;
    std::uint32_t payload_crc;
};

struct SharedCommandRegion {
    std::uint32_t magic;
    std::uint32_t version;
    SharedSlot<CommandSetpoint> setpoint;
    SharedSlot<CommandArm> arm;
    SharedSlot<Heartbeat> heartbeat;
    SharedSlot<CommandEmergencyStop> emergency_stop;
    SharedSlot<FlightConfig> config;
    SharedSlot<RtReport> rt_report;
};
#pragma pack(pop)

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= static_cast<std::uint32_t>(data[index]);
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = static_cast<std::uint32_t>(-(crc & 1u));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

template <typename T>
inline std::uint32_t payload_crc(const T& payload) {
    static_assert(std::is_trivially_copyable_v<T>);
    return crc32(reinterpret_cast<const std::uint8_t*>(&payload), sizeof(T));
}

template <typename T>
inline bool verify_slot(const SharedSlot<T>& slot) {
    if (slot.seq == 0u) {
        return false;
    }
    return slot.payload_crc == payload_crc(slot.payload);
}

inline std::uint16_t clamp_pwm(float pwm_us) {
    const auto bounded = std::clamp(pwm_us, static_cast<float>(kPwmMinUs), static_cast<float>(kPwmMaxUs));
    return static_cast<std::uint16_t>(bounded);
}

inline FlightConfig default_flight_config() {
    FlightConfig config{};
    config.angle_p = 5.0f;
    config.rate_p = 0.12f;
    config.rate_i = 0.02f;
    config.rate_d = 0.003f;
    config.throttle_expo = 0.45f;
    config.deadzone_us = 15u;
    config.reserved0 = 0u;
    config.update_seq = 1u;
    return config;
}

}  // namespace amp::proto
