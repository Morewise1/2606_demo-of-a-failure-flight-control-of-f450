#pragma once

#include <array>
#include <cstdint>

#include "../common/protocol.hpp"
#include "Estimator.hpp"

namespace amp::rt {

struct ControlSetpoint {
    float throttle;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
};

struct PidConfig {
    float kp;
    float ki;
    float kd;
    float integrator_min;
    float integrator_max;
    float output_min;
    float output_max;
};

class PidAxis {
public:
    explicit PidAxis(PidConfig config);
    float update(float error, float dt_s, float measured_value);
    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    PidConfig config_;
    float integrator_{0.0f};
    float previous_measurement_{0.0f};
    bool first_{true};
};

class Controller {
public:
    Controller();
    void setSetpoint(const ControlSetpoint& setpoint);
    void setArmState(proto::ArmState state);
    void applyConfig(const proto::FlightConfig& config);
    const proto::FlightConfig& activeConfig() const;
    std::array<std::uint16_t, proto::kMotorCount> update(const AttitudeState& state, float dt_s);
    proto::ArmState armState() const;

private:
    std::array<std::uint16_t, proto::kMotorCount> mix(float throttle, float pitch_out, float roll_out, float yaw_out) const;
    void resetIntegrators();

    ControlSetpoint setpoint_{0.0f, 0.0f, 0.0f, 0.0f};
    proto::ArmState arm_state_{proto::ArmState::Disarmed};
    PidAxis roll_angle_;
    PidAxis pitch_angle_;
    PidAxis roll_rate_;
    PidAxis pitch_rate_;
    PidAxis yaw_rate_;
    proto::FlightConfig active_config_{proto::default_flight_config()};
};

}  // namespace amp::rt
