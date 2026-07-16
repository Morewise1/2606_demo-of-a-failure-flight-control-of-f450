#include "Sense.hpp"

#include "RtosHal.hpp"

#include <cstdio>
#include <utility>

namespace amp::rt {

namespace {

constexpr std::uint8_t kRegSampleRateDiv = 0x19;
constexpr std::uint8_t kRegConfig = 0x1A;
constexpr std::uint8_t kRegGyroConfig = 0x1B;
constexpr std::uint8_t kRegAccelConfig = 0x1C;
constexpr std::uint8_t kRegAccelXoutH = 0x3B;
constexpr std::uint8_t kRegPwrMgmt1 = 0x6B;
constexpr std::uint8_t kRegWhoAmI = 0x75;

constexpr std::uint8_t kExpectedWhoAmI = 0x68;
constexpr float kGravity = 9.80665f;
constexpr float kAccelScaleLsbPerG = 8192.0f;
constexpr float kGyroScaleLsbPerDps = 65.5f;

}  // namespace

Mpu6050I2cBackend::Mpu6050I2cBackend(I2cBusConfig config) : config_(std::move(config)) {}

Mpu6050I2cBackend::~Mpu6050I2cBackend() = default;

bool Mpu6050I2cBackend::init() {
    if (config_.use_magnetometer && config_.mpu_addr == config_.mag_addr) {
        std::printf("[RT][IMU] I2C address conflict: MPU=0x%02X MAG=0x%02X\n", config_.mpu_addr, config_.mag_addr);
        return false;
    }

    if (hal_i2c_init(config_.bus_id, config_.bus_speed_hz) != 0) {
        std::printf("[RT][IMU] I2C%u init failed\n", config_.bus_id);
        return false;
    }

    std::uint8_t who_am_i = 0;
    if (!readRegisters(kRegWhoAmI, &who_am_i, 1)) {
        std::printf("[RT][IMU] MPU6050 WHO_AM_I read failed\n");
        return false;
    }
    if ((who_am_i & 0x7Eu) != kExpectedWhoAmI) {
        std::printf("[RT][IMU] unexpected WHO_AM_I=0x%02X\n", who_am_i);
        return false;
    }

    if (!writeRegister(kRegPwrMgmt1, 0x00)) {
        return false;
    }
    hal_delay_ms(100);

    const bool configured =
        writeRegister(kRegSampleRateDiv, 0x01) &&
        writeRegister(kRegConfig, 0x03) &&
        writeRegister(kRegGyroConfig, 0x08) &&
        writeRegister(kRegAccelConfig, 0x08);
    if (!configured) {
        std::printf("[RT][IMU] MPU6050 register configuration failed\n");
        return false;
    }
    return true;
}

bool Mpu6050I2cBackend::calibrate(std::size_t sample_count) {
    if (sample_count == 0) {
        return false;
    }

    std::array<float, 3> gyro_sum_dps{0.0f, 0.0f, 0.0f};
    std::size_t accepted = 0;
    for (std::size_t index = 0; index < sample_count; ++index) {
        RawImuSample sample{};
        if (read(sample)) {
            gyro_sum_dps[0] += sample.gyro_x_dps;
            gyro_sum_dps[1] += sample.gyro_y_dps;
            gyro_sum_dps[2] += sample.gyro_z_dps;
            ++accepted;
        }
        hal_delay_ms(2);
    }

    if (accepted == 0) {
        return false;
    }

    const float inv_count = 1.0f / static_cast<float>(accepted);
    bias_dps_[0] = gyro_sum_dps[0] * inv_count;
    bias_dps_[1] = gyro_sum_dps[1] * inv_count;
    bias_dps_[2] = gyro_sum_dps[2] * inv_count;
    std::printf("[RT][IMU] gyro bias dps=%.4f,%.4f,%.4f\n", bias_dps_[0], bias_dps_[1], bias_dps_[2]);
    return true;
}

bool Mpu6050I2cBackend::read(RawImuSample& sample) {
    std::uint8_t raw[14]{};
    if (!readRegisters(kRegAccelXoutH, raw, sizeof(raw))) {
        return false;
    }

    const std::int16_t acc_x = be16(raw[0], raw[1]);
    const std::int16_t acc_y = be16(raw[2], raw[3]);
    const std::int16_t acc_z = be16(raw[4], raw[5]);
    const std::int16_t gyro_x = be16(raw[8], raw[9]);
    const std::int16_t gyro_y = be16(raw[10], raw[11]);
    const std::int16_t gyro_z = be16(raw[12], raw[13]);

    sample.acc_x = static_cast<float>(acc_x) / kAccelScaleLsbPerG * kGravity;
    sample.acc_y = static_cast<float>(acc_y) / kAccelScaleLsbPerG * kGravity;
    sample.acc_z = static_cast<float>(acc_z) / kAccelScaleLsbPerG * kGravity;
    sample.gyro_x_dps = static_cast<float>(gyro_x) / kGyroScaleLsbPerDps;
    sample.gyro_y_dps = static_cast<float>(gyro_y) / kGyroScaleLsbPerDps;
    sample.gyro_z_dps = static_cast<float>(gyro_z) / kGyroScaleLsbPerDps;
    sample.timestamp_us = hal_micros();
    return true;
}

std::array<float, 3> Mpu6050I2cBackend::gyro_bias_dps() const {
    return bias_dps_;
}

bool Mpu6050I2cBackend::writeRegister(std::uint8_t reg, std::uint8_t value) {
    const std::uint8_t packet[2]{reg, value};
    return hal_i2c_write(config_.bus_id, config_.mpu_addr, packet, sizeof(packet)) == 0;
}

bool Mpu6050I2cBackend::readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length) {
    if (buffer == nullptr || length == 0) {
        return false;
    }
    if (hal_i2c_write(config_.bus_id, config_.mpu_addr, &start_reg, 1) != 0) {
        return false;
    }
    return hal_i2c_read(config_.bus_id, config_.mpu_addr, buffer, length) == 0;
}

std::int16_t Mpu6050I2cBackend::be16(std::uint8_t high, std::uint8_t low) {
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(high) << 8u) | static_cast<std::uint16_t>(low));
}

}  // namespace amp::rt
