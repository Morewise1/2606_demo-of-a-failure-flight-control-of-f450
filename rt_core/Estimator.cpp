#include "Estimator.hpp"

#include <algorithm>
#include <cmath>

namespace amp::rt {

AxisKalman::AxisKalman() = default;

float AxisKalman::update(float measured_angle_deg, float measured_rate_dps, float dt_s) {
    const float rate = measured_rate_dps - bias_dps_;
    angle_deg_ += dt_s * rate;

    p00_ += dt_s * (dt_s * p11_ - p01_ - p10_ + q_angle_);
    p01_ -= dt_s * p11_;
    p10_ -= dt_s * p11_;
    p11_ += q_bias_ * dt_s;

    const float innovation = measured_angle_deg - angle_deg_;
    const float innovation_cov = p00_ + r_measure_;
    const float k0 = p00_ / innovation_cov;
    const float k1 = p10_ / innovation_cov;

    angle_deg_ += k0 * innovation;
    bias_dps_ += k1 * innovation;

    const float p00_temp = p00_;
    const float p01_temp = p01_;

    p00_ -= k0 * p00_temp;
    p01_ -= k0 * p01_temp;
    p10_ -= k1 * p00_temp;
    p11_ -= k1 * p01_temp;

    return angle_deg_;
}

void Estimator::setGyroBias(const std::array<float, 3>& bias_dps) {
    gyro_bias_ = bias_dps;
}

AttitudeState Estimator::update(const RawImuSample& sample, float dt_s) {
    constexpr float kRadToDeg = 57.2957795f;

    const float roll_acc_deg = std::atan2(sample.acc_y, sample.acc_z) * kRadToDeg;
    const float pitch_acc_deg = std::atan2(-sample.acc_x, std::sqrt(sample.acc_y * sample.acc_y + sample.acc_z * sample.acc_z)) * kRadToDeg;

    const float roll_rate = sample.gyro_x_dps - gyro_bias_[0];
    const float pitch_rate = sample.gyro_y_dps - gyro_bias_[1];
    const float yaw_rate = sample.gyro_z_dps - gyro_bias_[2];

    last_state_.roll_deg = roll_filter_.update(roll_acc_deg, roll_rate, dt_s);
    last_state_.pitch_deg = pitch_filter_.update(pitch_acc_deg, pitch_rate, dt_s);
    last_state_.yaw_deg += yaw_rate * dt_s;
    last_state_.roll_rate_dps = roll_rate;
    last_state_.pitch_rate_dps = pitch_rate;
    last_state_.yaw_rate_dps = yaw_rate;
    return last_state_;
}

AttitudeState Estimator::correctYawFromMag(const RawMagSample& sample, float dt_s) {
    constexpr float kDegToRad = 0.0174532925f;
    constexpr float kRadToDeg = 57.2957795f;

    if (dt_s <= 0.0f) {
        return last_state_;
    }

    const float roll_rad = last_state_.roll_deg * kDegToRad;
    const float pitch_rad = last_state_.pitch_deg * kDegToRad;
    const float cos_roll = std::cos(roll_rad);
    const float sin_roll = std::sin(roll_rad);
    const float cos_pitch = std::cos(pitch_rad);
    const float sin_pitch = std::sin(pitch_rad);

    const float xh = sample.mag_x * cos_pitch + sample.mag_z * sin_pitch;
    const float yh = sample.mag_x * sin_roll * sin_pitch + sample.mag_y * cos_roll - sample.mag_z * sin_roll * cos_pitch;
    const float heading_deg = wrapAngleDeg(std::atan2(-yh, xh) * kRadToDeg);

    const float error = wrapAngleDeg(heading_deg - last_state_.yaw_deg);
    const float correction_alpha = std::clamp(dt_s * 1.5f, 0.001f, 0.03f);
    last_state_.yaw_deg = wrapAngleDeg(last_state_.yaw_deg + error * correction_alpha);
    return last_state_;
}

const AttitudeState& Estimator::last() const {
    return last_state_;
}

float Estimator::wrapAngleDeg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

}  // namespace amp::rt
