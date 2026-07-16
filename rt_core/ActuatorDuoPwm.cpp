#include "ActuatorDuoPwm.hpp"

#include <cstddef>

namespace amp::rt {

bool ActuatorDuoPwm::init() {
    status_ = ActuatorDuoPwmStatus{};
    status_.map_ok = mapMatchesBringup();
    return status_.map_ok;
}

ActuatorDuoPwmStatus ActuatorDuoPwm::update(
    const std::array<std::uint16_t, proto::kMotorCount>& mix_us,
    bool output_allowed,
    bool rc_valid,
    bool sensor_ready,
    bool emergency_latched) {
    status_.requested_us = mix_us;
    status_.map_ok = mapMatchesBringup();
    status_.safe_forced = !output_allowed || !rc_valid || !sensor_ready || emergency_latched;

    if (status_.safe_forced) {
        status_.output_us = {
            kPwmSafeUs,
            kPwmSafeUs,
            kPwmSafeUs,
            kPwmSafeUs,
        };
        return status_;
    }

    for (std::size_t i = 0; i < proto::kMotorCount; ++i) {
        status_.output_us[i] = applyTrimAndMinSpin(i, mix_us[i]);
    }

    return status_;
}

const ActuatorDuoPwmStatus& ActuatorDuoPwm::status() const {
    return status_;
}

bool ActuatorDuoPwm::mapMatchesBringup() {
    for (std::size_t i = 0; i < proto::kMotorCount; ++i) {
        if (kMotorMap[i].pwm_id != kPwmIdMap[i] ||
            kMotorMap[i].channel != kPwmChannelMap[i]) {
            return false;
        }
    }
    return kPwmIdMap[0] == 1u && kPwmChannelMap[0] == 3u &&
           kPwmIdMap[1] == 1u && kPwmChannelMap[1] == 1u &&
           kPwmIdMap[2] == 1u && kPwmChannelMap[2] == 0u &&
           kPwmIdMap[3] == 1u && kPwmChannelMap[3] == 2u;
}

std::uint16_t ActuatorDuoPwm::clampPwm(std::int32_t us) {
    if (us < static_cast<std::int32_t>(kPwmSafeUs)) {
        return kPwmSafeUs;
    }
    if (us > static_cast<std::int32_t>(kPwmMaxUs)) {
        return kPwmMaxUs;
    }
    return static_cast<std::uint16_t>(us);
}

std::uint16_t ActuatorDuoPwm::applyTrimAndMinSpin(std::size_t index, std::uint16_t mix_us) {
    if (index >= proto::kMotorCount) {
        return kPwmSafeUs;
    }

    std::int32_t us = static_cast<std::int32_t>(mix_us) +
                      static_cast<std::int32_t>(kTrimUs[index]);
    if (mix_us > kPwmSafeUs &&
        us < static_cast<std::int32_t>(kMinSpinUs[index])) {
        us = static_cast<std::int32_t>(kMinSpinUs[index]);
    }

    return clampPwm(us);
}

}  // namespace amp::rt

extern "C" std::uint32_t amp_stage3_duo_pwm_map_ok() {
    return amp::rt::ActuatorDuoPwm::mapMatchesBringup() ? 1u : 0u;
}

