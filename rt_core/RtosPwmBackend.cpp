#include "Actuator.hpp"

#include "RtosHal.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace amp::rt {

RtosPwmBackend::RtosPwmBackend(PwmOutputConfig config) : config_(std::move(config)) {}

bool RtosPwmBackend::init() {
    if (hasDuplicateChannels()) {
        std::printf("[RT][PWM] duplicate output channel detected\n");
        return false;
    }

    for (std::size_t index = 0; index < proto::kMotorCount; ++index) {
        if (!configureChannel(index, proto::kPwmMinUs)) {
            std::printf("[RT][PWM] init failed on %s channel=%d\n", config_.labels[index], config_.channels[index]);
            return false;
        }
    }
    return true;
}

bool RtosPwmBackend::write(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) {
    for (std::size_t index = 0; index < proto::kMotorCount; ++index) {
        if (!setChannelPwm(index, pwm_us[index])) {
            return false;
        }
    }
    return true;
}

bool RtosPwmBackend::configureChannel(std::size_t index, std::uint16_t init_pwm_us) {
    const int channel = config_.channels[index];
    if (channel < 0) {
        return false;
    }

    const auto hardware_channel = static_cast<std::uint32_t>(channel);
    if (hal_pwm_init(hardware_channel, config_.period_ns) != 0) {
        return false;
    }
    if (hal_pwm_enable(hardware_channel, false) != 0) {
        return false;
    }
    if (!setChannelPwm(index, init_pwm_us)) {
        return false;
    }
    return hal_pwm_enable(hardware_channel, true) == 0;
}

bool RtosPwmBackend::setChannelPwm(std::size_t index, std::uint16_t pwm_us) {
    const int channel = config_.channels[index];
    if (channel < 0) {
        return false;
    }

    const auto hardware_channel = static_cast<std::uint32_t>(channel);
    const std::uint16_t bounded = std::clamp<std::uint16_t>(pwm_us, proto::kPwmMinUs, proto::kPwmMaxUs);
    const std::uint32_t high_time_ns = static_cast<std::uint32_t>(bounded) * 1000u;
    return hal_pwm_set_duty(hardware_channel, high_time_ns) == 0;
}

bool RtosPwmBackend::hasDuplicateChannels() const {
    for (std::size_t left = 0; left < proto::kMotorCount; ++left) {
        for (std::size_t right = left + 1; right < proto::kMotorCount; ++right) {
            if (config_.channels[left] == config_.channels[right]) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace amp::rt
