#include "Sense.hpp"

#include "RtosHal.hpp"

#include <cstdio>
#include <utility>

namespace amp::rt {

namespace {

constexpr std::uint8_t kRegConfigA = 0x00;
constexpr std::uint8_t kRegConfigB = 0x01;
constexpr std::uint8_t kRegMode = 0x02;
constexpr std::uint8_t kRegDataXMsb = 0x03;
constexpr std::uint8_t kRegIdA = 0x0A;
constexpr std::uint8_t kRegIdB = 0x0B;
constexpr std::uint8_t kRegIdC = 0x0C;

constexpr float kLsbPerGauss = 1090.0f;
constexpr float kMicroTeslaPerGauss = 100.0f;

}  // namespace

Hmc5883lI2cBackend::Hmc5883lI2cBackend(Hmc5883lConfig config) : config_(std::move(config)) {}

Hmc5883lI2cBackend::~Hmc5883lI2cBackend() = default;

bool Hmc5883lI2cBackend::init() {
    if (hal_i2c_init(config_.bus_id, config_.bus_speed_hz) != 0) {
        std::printf("[RT][MAG] I2C%u init failed\n", config_.bus_id);
        return false;
    }

    std::uint8_t id[3]{};
    if (!readRegisters(kRegIdA, id, sizeof(id))) {
        std::printf("[RT][MAG] HMC5883L id read failed\n");
        return false;
    }
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        std::printf("[RT][MAG] unexpected HMC id=%02X,%02X,%02X\n", id[0], id[1], id[2]);
        return false;
    }

    const bool configured =
        writeRegister(kRegConfigA, 0x78) &&
        writeRegister(kRegConfigB, 0x20) &&
        writeRegister(kRegMode, 0x00);
    if (!configured) {
        std::printf("[RT][MAG] HMC5883L register configuration failed\n");
        return false;
    }
    return true;
}

bool Hmc5883lI2cBackend::read(RawMagSample& sample) {
    std::uint8_t raw[6]{};
    if (!readRegisters(kRegDataXMsb, raw, sizeof(raw))) {
        return false;
    }

    const std::int16_t mag_x = be16(raw[0], raw[1]);
    const std::int16_t mag_z = be16(raw[2], raw[3]);
    const std::int16_t mag_y = be16(raw[4], raw[5]);

    sample.mag_x = static_cast<float>(mag_x) / kLsbPerGauss * kMicroTeslaPerGauss;
    sample.mag_y = static_cast<float>(mag_y) / kLsbPerGauss * kMicroTeslaPerGauss;
    sample.mag_z = static_cast<float>(mag_z) / kLsbPerGauss * kMicroTeslaPerGauss;
    sample.timestamp_us = hal_micros();
    return true;
}

bool Hmc5883lI2cBackend::writeRegister(std::uint8_t reg, std::uint8_t value) {
    const std::uint8_t packet[2]{reg, value};
    return hal_i2c_write(config_.bus_id, config_.address, packet, sizeof(packet)) == 0;
}

bool Hmc5883lI2cBackend::readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length) {
    if (buffer == nullptr || length == 0) {
        return false;
    }
    if (hal_i2c_write(config_.bus_id, config_.address, &start_reg, 1) != 0) {
        return false;
    }
    return hal_i2c_read(config_.bus_id, config_.address, buffer, length) == 0;
}

std::int16_t Hmc5883lI2cBackend::be16(std::uint8_t high, std::uint8_t low) {
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(high) << 8u) | static_cast<std::uint16_t>(low));
}

}  // namespace amp::rt
