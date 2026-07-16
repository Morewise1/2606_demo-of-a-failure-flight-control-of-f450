#include "FlightRtNodeStage3.hpp"

namespace amp::rt {

bool FlightRtNodeStage3::init() {
    step_count_ = 0;
    last_output_ = FlightRtNodeStage3Output{};
    return actuator_.init();
}

FlightRtNodeStage3Output FlightRtNodeStage3::step(const FlightRtNodeStage3Input& input) {
    ++step_count_;

    const bool output_allowed =
        input.armed && input.output_enabled && input.rc_valid && input.sensor_ready;
    const ActuatorDuoPwmStatus status =
        actuator_.update(input.mix_us,
                         output_allowed,
                         input.rc_valid,
                         input.sensor_ready,
                         input.emergency_latched);

    last_output_.motor_us = status.output_us;
    last_output_.safe_forced = status.safe_forced;
    last_output_.actuator_map_ok = status.map_ok;
    last_output_.step_count = step_count_;
    return last_output_;
}

const FlightRtNodeStage3Output& FlightRtNodeStage3::lastOutput() const {
    return last_output_;
}

}  // namespace amp::rt

