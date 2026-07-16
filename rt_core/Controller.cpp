#include "Controller.hpp"

#include <algorithm>

namespace amp::rt {

PidAxis::PidAxis(PidConfig config) : config_(config) {}

float PidAxis::update(float error, float dt_s, float measured_value) {
    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    integrator_ += error * dt_s * config_.ki;
    integrator_ = std::clamp(integrator_, config_.integrator_min, config_.integrator_max);

    const float derivative = first_ ? 0.0f : (previous_measurement_ - measured_value) / dt_s;
    first_ = false;
    previous_measurement_ = measured_value;

    float output = config_.kp * error + integrator_ + config_.kd * derivative;
    output = std::clamp(output, config_.output_min, config_.output_max);
    return output;
}

void PidAxis::reset() {
    integrator_ = 0.0f;
    previous_measurement_ = 0.0f;
    first_ = true;
}

void PidAxis::setTunings(float kp, float ki, float kd) {
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
}

Controller::Controller()
    : roll_angle_(PidConfig{4.0f, 0.0f, 0.05f, -50.0f, 50.0f, -220.0f, 220.0f}),
      pitch_angle_(PidConfig{4.0f, 0.0f, 0.05f, -50.0f, 50.0f, -220.0f, 220.0f}),
      roll_rate_(PidConfig{0.12f, 0.02f, 0.003f, -120.0f, 120.0f, -1.0f, 1.0f}),
      pitch_rate_(PidConfig{0.12f, 0.02f, 0.003f, -120.0f, 120.0f, -1.0f, 1.0f}),
      yaw_rate_(PidConfig{0.12f, 0.02f, 0.003f, -100.0f, 100.0f, -1.0f, 1.0f}) {
    applyConfig(active_config_);
}

void Controller::setSetpoint(const ControlSetpoint& setpoint) {
    setpoint_ = setpoint;
    setpoint_.throttle = std::clamp(setpoint_.throttle, 0.0f, 1.0f);
    setpoint_.roll_deg = std::clamp(setpoint_.roll_deg, -35.0f, 35.0f);
    setpoint_.pitch_deg = std::clamp(setpoint_.pitch_deg, -35.0f, 35.0f);
    setpoint_.yaw_rate_dps = std::clamp(setpoint_.yaw_rate_dps, -180.0f, 180.0f);
}

void Controller::setArmState(proto::ArmState state) {
    if (state == arm_state_) {
        return;
    }
    arm_state_ = state;
    resetIntegrators();
}

void Controller::applyConfig(const proto::FlightConfig& config) {
    active_config_ = config;
    active_config_.angle_p = std::clamp(active_config_.angle_p, 0.5f, 20.0f);
    active_config_.rate_p = std::clamp(active_config_.rate_p, 0.01f, 1.0f);
    active_config_.rate_i = std::clamp(active_config_.rate_i, 0.0f, 1.0f);
    active_config_.rate_d = std::clamp(active_config_.rate_d, 0.0f, 0.1f);
    roll_angle_.setTunings(active_config_.angle_p, 0.0f, 0.05f);
    pitch_angle_.setTunings(active_config_.angle_p, 0.0f, 0.05f);
    roll_rate_.setTunings(active_config_.rate_p, active_config_.rate_i, active_config_.rate_d);
    pitch_rate_.setTunings(active_config_.rate_p, active_config_.rate_i, active_config_.rate_d);
    yaw_rate_.setTunings(active_config_.rate_p, active_config_.rate_i, active_config_.rate_d);
    resetIntegrators();
}

const proto::FlightConfig& Controller::activeConfig() const {
    return active_config_;
}

std::array<std::uint16_t, proto::kMotorCount> Controller::update(const AttitudeState& state, float dt_s) {
    if (arm_state_ != proto::ArmState::Armed) {
        return {proto::kPwmMinUs, proto::kPwmMinUs, proto::kPwmMinUs, proto::kPwmMinUs};
    }

    const float roll_angle_error = setpoint_.roll_deg - state.roll_deg;
    const float pitch_angle_error = setpoint_.pitch_deg - state.pitch_deg;

    const float target_roll_rate = roll_angle_.update(roll_angle_error, dt_s, state.roll_deg);
    const float target_pitch_rate = pitch_angle_.update(pitch_angle_error, dt_s, state.pitch_deg);

    const float roll_rate_error = target_roll_rate - state.roll_rate_dps;
    const float pitch_rate_error = target_pitch_rate - state.pitch_rate_dps;
    const float yaw_rate_error = setpoint_.yaw_rate_dps - state.yaw_rate_dps;

    const float roll_out = roll_rate_.update(roll_rate_error, dt_s, state.roll_rate_dps);
    const float pitch_out = pitch_rate_.update(pitch_rate_error, dt_s, state.pitch_rate_dps);
    const float yaw_out = yaw_rate_.update(yaw_rate_error, dt_s, state.yaw_rate_dps);
    return mix(setpoint_.throttle, pitch_out, roll_out, yaw_out);
}

proto::ArmState Controller::armState() const {
    return arm_state_;
}

std::array<std::uint16_t, proto::kMotorCount> Controller::mix(float throttle, float pitch_out, float roll_out, float yaw_out) const {
    const float base = static_cast<float>(proto::kPwmMinUs) + throttle * 1000.0f;
    constexpr float kMixScaleUs = 320.0f;

    const float motor1 = base + (+pitch_out - roll_out - yaw_out) * kMixScaleUs;
    const float motor2 = base + (-pitch_out - roll_out + yaw_out) * kMixScaleUs;
    const float motor3 = base + (-pitch_out + roll_out - yaw_out) * kMixScaleUs;
    const float motor4 = base + (+pitch_out + roll_out + yaw_out) * kMixScaleUs;

    return {
        proto::clamp_pwm(motor1),
        proto::clamp_pwm(motor2),
        proto::clamp_pwm(motor3),
        proto::clamp_pwm(motor4),
    };
}

void Controller::resetIntegrators() {
    roll_angle_.reset();
    pitch_angle_.reset();
    roll_rate_.reset();
    pitch_rate_.reset();
    yaw_rate_.reset();
}

}  // namespace amp::rt
