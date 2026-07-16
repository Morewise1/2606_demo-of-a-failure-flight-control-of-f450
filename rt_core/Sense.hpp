#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace amp::rt {

struct RawImuSample {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    std::uint64_t timestamp_us;
};

struct RawMagSample {
    float mag_x;
    float mag_y;
    float mag_z;
    std::uint64_t timestamp_us;
};

class ImuBackend {
public:
    virtual ~ImuBackend() = default;
    virtual bool init() = 0;
    virtual bool calibrate(std::size_t sample_count) = 0;
    virtual bool read(RawImuSample& sample) = 0;
    virtual std::array<float, 3> gyro_bias_dps() const = 0;
};

class MagBackend {
public:
    virtual ~MagBackend() = default;
    virtual bool init() = 0;
    virtual bool read(RawMagSample& sample) = 0;
};

class SimulatedImuBackend final : public ImuBackend {
public:
    bool init() override;
    bool calibrate(std::size_t sample_count) override;
    bool read(RawImuSample& sample) override;
    std::array<float, 3> gyro_bias_dps() const override;

private:
    std::array<float, 3> bias_dps_{0.25f, -0.21f, 0.16f};
    std::uint64_t timestamp_us_{0};
    float phase_{0.0f};
};

class SimulatedMagBackend final : public MagBackend {
public:
    bool init() override;
    bool read(RawMagSample& sample) override;

private:
    std::uint64_t timestamp_us_{0};
};

struct I2cBusConfig {
    std::uint8_t bus_id{0};
    std::uint32_t bus_speed_hz{400000};
    std::uint8_t mpu_addr{0x68};
    std::uint8_t mag_addr{0x1E};
    bool use_magnetometer{true};
};

struct Hmc5883lConfig {
    std::uint8_t bus_id{0};
    std::uint32_t bus_speed_hz{400000};
    std::uint8_t address{0x1E};
};

class Mpu6050I2cBackend final : public ImuBackend {
public:
    explicit Mpu6050I2cBackend(I2cBusConfig config = {});
    ~Mpu6050I2cBackend() override;

    bool init() override;
    bool calibrate(std::size_t sample_count) override;
    bool read(RawImuSample& sample) override;
    std::array<float, 3> gyro_bias_dps() const override;

private:
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length);
    static std::int16_t be16(std::uint8_t high, std::uint8_t low);

    I2cBusConfig config_;
    std::array<float, 3> bias_dps_{0.0f, 0.0f, 0.0f};
};

class Hmc5883lI2cBackend final : public MagBackend {
public:
    explicit Hmc5883lI2cBackend(Hmc5883lConfig config = {});
    ~Hmc5883lI2cBackend() override;

    bool init() override;
    bool read(RawMagSample& sample) override;

private:
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length);
    static std::int16_t be16(std::uint8_t high, std::uint8_t low);

    Hmc5883lConfig config_;
};

class Sense {
public:
    explicit Sense(std::unique_ptr<ImuBackend> backend, std::unique_ptr<MagBackend> mag_backend = nullptr, bool require_mag = false);
    bool init();
    bool sample(RawImuSample& sample);
    bool sampleMag(RawMagSample& sample);
    bool hasMagnetometer() const;
    std::array<float, 3> gyro_bias_dps() const;

private:
    std::unique_ptr<ImuBackend> backend_;
    std::unique_ptr<MagBackend> mag_backend_;
    bool require_mag_{false};
    bool mag_ready_{false};
};

}  // namespace amp::rt
