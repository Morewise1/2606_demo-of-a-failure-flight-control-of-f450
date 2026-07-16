#pragma once

#include <array>

#include "Sense.hpp"

namespace amp::rt {

struct AttitudeState {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
};

class AxisKalman {
public:
    AxisKalman();
    float update(float measured_angle_deg, float measured_rate_dps, float dt_s);

private:
    float angle_deg_{0.0f};
    float bias_dps_{0.0f};
    float p00_{1.0f};
    float p01_{0.0f};
    float p10_{0.0f};
    float p11_{1.0f};
    float q_angle_{0.001f};
    float q_bias_{0.003f};
    float r_measure_{0.03f};
};

class Estimator {
public:
    void setGyroBias(const std::array<float, 3>& bias_dps);
    AttitudeState update(const RawImuSample& sample, float dt_s);
    AttitudeState correctYawFromMag(const RawMagSample& sample, float dt_s);
    const AttitudeState& last() const;

private:
    static float wrapAngleDeg(float angle_deg);

    std::array<float, 3> gyro_bias_{0.0f, 0.0f, 0.0f};
    AxisKalman roll_filter_;
    AxisKalman pitch_filter_;
    AttitudeState last_state_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace amp::rt
