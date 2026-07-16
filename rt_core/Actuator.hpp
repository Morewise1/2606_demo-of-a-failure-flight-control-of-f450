#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "../common/protocol.hpp"

namespace amp::rt {

class MotorBackend {
public:
    virtual ~MotorBackend() = default;
    virtual bool init() = 0;
    virtual bool write(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) = 0;
};

class LoggingMotorBackend final : public MotorBackend {
public:
    bool init() override;
    bool write(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) override;
};

struct PwmOutputConfig {
    std::array<int, proto::kMotorCount> channels{0, 1, 2, 3};
    std::array<const char*, proto::kMotorCount> labels{"PWM0", "PWM1", "PWM2", "PWM3"};
    std::uint32_t period_ns{2500000};
};

class RtosPwmBackend final : public MotorBackend {
public:
    explicit RtosPwmBackend(PwmOutputConfig config = {});
    bool init() override;
    bool write(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us) override;

private:
    bool configureChannel(std::size_t index, std::uint16_t init_pwm_us);
    bool setChannelPwm(std::size_t index, std::uint16_t pwm_us);
    bool hasDuplicateChannels() const;

    PwmOutputConfig config_;
};

class Actuator {
public:
    explicit Actuator(std::unique_ptr<MotorBackend> backend);
    bool init();
    bool output(const std::array<std::uint16_t, proto::kMotorCount>& pwm_us);
    bool stop();
    const std::array<std::uint16_t, proto::kMotorCount>& lastPwm() const;

private:
    std::unique_ptr<MotorBackend> backend_;
    std::array<std::uint16_t, proto::kMotorCount> last_pwm_{
        proto::kPwmMinUs,
        proto::kPwmMinUs,
        proto::kPwmMinUs,
        proto::kPwmMinUs,
    };
};

}  // namespace amp::rt
