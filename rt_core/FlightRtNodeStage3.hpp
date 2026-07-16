#pragma once

#include <array>
#include <cstdint>

#include "../common/protocol.hpp"
#include "ActuatorDuoPwm.hpp"

namespace amp::rt {

struct FlightRtNodeStage3Input {
    std::array<std::uint16_t, proto::kMotorCount> mix_us{
        1000u,
        1000u,
        1000u,
        1000u,
    };
    bool rc_valid{false};
    bool sensor_ready{false};
    bool armed{false};
    bool output_enabled{false};
    bool emergency_latched{false};
};

struct FlightRtNodeStage3Output {
    std::array<std::uint16_t, proto::kMotorCount> motor_us{
        1000u,
        1000u,
        1000u,
        1000u,
    };
    bool safe_forced{true};
    bool actuator_map_ok{false};
    std::uint32_t step_count{0};
};

class FlightRtNodeStage3 final {
public:
    bool init();
    FlightRtNodeStage3Output step(const FlightRtNodeStage3Input& input);
    const FlightRtNodeStage3Output& lastOutput() const;

private:
    ActuatorDuoPwm actuator_{};
    FlightRtNodeStage3Output last_output_{};
    std::uint32_t step_count_{0};
};

}  // namespace amp::rt

