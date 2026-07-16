#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "../common/protocol.hpp"

namespace amp::rt {

struct DuoPwmMotorMapEntry {
    const char* name;
    const char* gp;
    std::uint8_t pwm_number;
    std::uint8_t pwm_id;
    std::uint8_t channel;
};

struct ActuatorDuoPwmStatus {
    std::array<std::uint16_t, proto::kMotorCount> requested_us{
        1000u,
        1000u,
        1000u,
        1000u,
    };
    std::array<std::uint16_t, proto::kMotorCount> output_us{
        1000u,
        1000u,
        1000u,
        1000u,
    };
    bool safe_forced{true};
    bool map_ok{false};
};

class ActuatorDuoPwm final {
public:
    static constexpr std::uint16_t kPwmSafeUs = 1000u;
    static constexpr std::uint16_t kPwmMaxUs = 1500u;
    static constexpr std::array<std::uint8_t, proto::kMotorCount> kPwmIdMap{
        1u, 1u, 1u, 1u,
    };
    static constexpr std::array<std::uint8_t, proto::kMotorCount> kPwmChannelMap{
        3u, 1u, 0u, 2u,
    };
    static constexpr std::array<std::uint16_t, proto::kMotorCount> kMinSpinUs{
        1220u, 1235u, 1220u, 1220u,
    };
    static constexpr std::array<std::int16_t, proto::kMotorCount> kTrimUs{
        10, 0, 0, 10,
    };
    static constexpr std::array<DuoPwmMotorMapEntry, proto::kMotorCount> kMotorMap{{
        {"motor0 LF", "GP2", 7u, 1u, 3u},
        {"motor1 RF", "GP4", 5u, 1u, 1u},
        {"motor2 RR", "GP9", 4u, 1u, 0u},
        {"motor3 LB", "GP3", 6u, 1u, 2u},
    }};

    bool init();
    ActuatorDuoPwmStatus update(const std::array<std::uint16_t, proto::kMotorCount>& mix_us,
                                bool output_allowed,
                                bool rc_valid,
                                bool sensor_ready,
                                bool emergency_latched);
    const ActuatorDuoPwmStatus& status() const;

    static bool mapMatchesBringup();

private:
    static std::uint16_t clampPwm(std::int32_t us);
    static std::uint16_t applyTrimAndMinSpin(std::size_t index, std::uint16_t mix_us);

    ActuatorDuoPwmStatus status_{};
};

}  // namespace amp::rt
