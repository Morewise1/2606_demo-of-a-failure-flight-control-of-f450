#include "Actuator.hpp"

#include <cstdio>
#include <utility>

namespace amp::rt {

bool LoggingMotorBackend::init() {
    return true;
}

bool LoggingMotorBackend::write(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) {
    static std::uint32_t count = 0;
    ++count;
    if ((count % 120u) == 0u) {
        std::printf("[RT] pwm=%u,%u,%u,%u\n", pwm_us[0], pwm_us[1], pwm_us[2], pwm_us[3]);
    }
    return true;
}

Actuator::Actuator(std::unique_ptr<MotorBackend> backend) : backend_(std::move(backend)) {}

bool Actuator::init() {
    if (!backend_) {
        return false;
    }
    return backend_->init() && stop();
}

bool Actuator::output(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) {
    if (!backend_) {
        return false;
    }
    last_pwm_ = pwm_us;
    return backend_->write(last_pwm_);
}

bool Actuator::stop() {
    return output({proto::kPwmMinUs, proto::kPwmMinUs, proto::kPwmMinUs, proto::kPwmMinUs});
}

const std::array<std::uint16_t, proto::kMotorCount>& Actuator::lastPwm() const {
    return last_pwm_;
}

}  // namespace amp::rt
