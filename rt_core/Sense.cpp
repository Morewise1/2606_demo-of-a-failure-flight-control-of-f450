#include "Sense.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

namespace amp::rt {

bool SimulatedImuBackend::init() {
    timestamp_us_ = 0;
    phase_ = 0.0f;
    return true;
}

bool SimulatedImuBackend::calibrate(std::size_t sample_count) {
    if (sample_count == 0) {
        return false;
    }
    return true;
}

bool SimulatedImuBackend::read(RawImuSample& sample) {
    constexpr float kDt = 0.0025f;
    constexpr float kRadToDeg = 57.2957795f;
    constexpr float kGravity = 9.80665f;

    phase_ += 2.0f * 3.1415926f * 0.65f * kDt;
    timestamp_us_ += 2500;

    const float roll_rad = 0.12f * std::sin(phase_);
    const float pitch_rad = 0.09f * std::sin(phase_ * 0.7f);
    const float yaw_rate_rad = 0.6f * std::cos(phase_ * 0.4f);

    sample.acc_x = -kGravity * std::sin(pitch_rad);
    sample.acc_y = kGravity * std::sin(roll_rad) * std::cos(pitch_rad);
    sample.acc_z = kGravity * std::cos(roll_rad) * std::cos(pitch_rad);
    sample.gyro_x_dps = 0.12f * 0.65f * 2.0f * 3.1415926f * std::cos(phase_) * kRadToDeg + bias_dps_[0];
    sample.gyro_y_dps = 0.09f * 0.7f * 2.0f * 3.1415926f * std::cos(phase_ * 0.7f) * kRadToDeg + bias_dps_[1];
    sample.gyro_z_dps = yaw_rate_rad * kRadToDeg + bias_dps_[2];
    sample.timestamp_us = timestamp_us_;
    return true;
}

std::array<float, 3> SimulatedImuBackend::gyro_bias_dps() const {
    return bias_dps_;
}

bool SimulatedMagBackend::init() {
    timestamp_us_ = 0;
    return true;
}

bool SimulatedMagBackend::read(RawMagSample& sample) {
    timestamp_us_ += 20000;
    sample.mag_x = 1.0f;
    sample.mag_y = 0.0f;
    sample.mag_z = 0.0f;
    sample.timestamp_us = timestamp_us_;
    return true;
}

Sense::Sense(std::unique_ptr<ImuBackend> backend, std::unique_ptr<MagBackend> mag_backend, bool require_mag)
    : backend_(std::move(backend)), mag_backend_(std::move(mag_backend)), require_mag_(require_mag) {}

bool Sense::init() {
    if (!backend_) {
        return false;
    }
    if (!(backend_->init() && backend_->calibrate(400))) {
        return false;
    }
    if (mag_backend_) {
        mag_ready_ = mag_backend_->init();
        if (!mag_ready_ && require_mag_) {
            return false;
        }
    }
    return true;
}

bool Sense::sample(RawImuSample& sample) {
    if (!backend_) {
        return false;
    }
    return backend_->read(sample);
}

bool Sense::sampleMag(RawMagSample& sample) {
    if (!mag_backend_ || !mag_ready_) {
        return false;
    }
    return mag_backend_->read(sample);
}

bool Sense::hasMagnetometer() const {
    return mag_ready_;
}

std::array<float, 3> Sense::gyro_bias_dps() const {
    if (!backend_) {
        return {0.0f, 0.0f, 0.0f};
    }
    return backend_->gyro_bias_dps();
}

}  // namespace amp::rt
