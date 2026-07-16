#include "Sense.hpp"

#include "RtosHal.hpp"

#include <cstdio>

namespace {

constexpr std::uint8_t kDiagI2cBus = 0;
constexpr std::uint8_t kEspUart = 2;
constexpr std::uint32_t kDiagI2cSpeedHz = 400000;
constexpr std::uint32_t kEspBaudRate = 115200;
constexpr std::uint8_t kMpuAddr = 0x68;
constexpr std::uint8_t kHmcAddr = 0x1E;

amp::rt::I2cBusConfig makeMpuConfig() {
    amp::rt::I2cBusConfig config{};
    config.bus_id = kDiagI2cBus;
    config.bus_speed_hz = kDiagI2cSpeedHz;
    config.mpu_addr = kMpuAddr;
    config.mag_addr = kHmcAddr;
    config.use_magnetometer = true;
    return config;
}

amp::rt::Hmc5883lConfig makeHmcConfig() {
    amp::rt::Hmc5883lConfig config{};
    config.bus_id = kDiagI2cBus;
    config.bus_speed_hz = kDiagI2cSpeedHz;
    config.address = kHmcAddr;
    return config;
}

void idleForever() {
    for (;;) {
        hal_delay_ms(1000);
    }
}

}  // namespace

extern "C" {

void mpu6050_probe_task(void* pvParameters) {
    (void)pvParameters;

    amp::rt::Mpu6050I2cBackend mpu(makeMpuConfig());
    if (!mpu.init()) {
        std::printf("[DIAG][MPU] init failed\n");
        idleForever();
    }
    if (!mpu.calibrate(200)) {
        std::printf("[DIAG][MPU] calibration failed\n");
        idleForever();
    }

    for (;;) {
        amp::rt::RawImuSample sample{};
        if (mpu.read(sample)) {
            std::printf("[DIAG][MPU] acc=%.3f,%.3f,%.3f gyro=%.3f,%.3f,%.3f t=%llu\n",
                        sample.acc_x,
                        sample.acc_y,
                        sample.acc_z,
                        sample.gyro_x_dps,
                        sample.gyro_y_dps,
                        sample.gyro_z_dps,
                        static_cast<unsigned long long>(sample.timestamp_us));
        } else {
            std::printf("[DIAG][MPU] read failed\n");
        }
        hal_delay_ms(100);
    }
}

void hmc5883l_probe_task(void* pvParameters) {
    (void)pvParameters;

    amp::rt::Hmc5883lI2cBackend hmc(makeHmcConfig());
    if (!hmc.init()) {
        std::printf("[DIAG][HMC] init failed\n");
        idleForever();
    }

    for (;;) {
        amp::rt::RawMagSample sample{};
        if (hmc.read(sample)) {
            std::printf("[DIAG][HMC] mag_uT=%.3f,%.3f,%.3f t=%llu\n",
                        sample.mag_x,
                        sample.mag_y,
                        sample.mag_z,
                        static_cast<unsigned long long>(sample.timestamp_us));
        } else {
            std::printf("[DIAG][HMC] read failed\n");
        }
        hal_delay_ms(100);
    }
}

void sensor_probe_task(void* pvParameters) {
    (void)pvParameters;

    amp::rt::Mpu6050I2cBackend mpu(makeMpuConfig());
    amp::rt::Hmc5883lI2cBackend hmc(makeHmcConfig());
    const bool mpu_ok = mpu.init() && mpu.calibrate(200);
    const bool hmc_ok = hmc.init();
    std::printf("[DIAG] mpu=%s hmc=%s\n", mpu_ok ? "ok" : "fail", hmc_ok ? "ok" : "fail");

    for (;;) {
        if (mpu_ok) {
            amp::rt::RawImuSample imu{};
            if (mpu.read(imu)) {
                std::printf("[DIAG][MPU] acc=%.2f,%.2f,%.2f gyro=%.2f,%.2f,%.2f\n",
                            imu.acc_x,
                            imu.acc_y,
                            imu.acc_z,
                            imu.gyro_x_dps,
                            imu.gyro_y_dps,
                            imu.gyro_z_dps);
            }
        }
        if (hmc_ok) {
            amp::rt::RawMagSample mag{};
            if (hmc.read(mag)) {
                std::printf("[DIAG][HMC] mag_uT=%.2f,%.2f,%.2f\n", mag.mag_x, mag.mag_y, mag.mag_z);
            }
        }
        hal_delay_ms(200);
    }
}

void esp8266_probe_task(void* pvParameters) {
    (void)pvParameters;

    if (amp_hal_uart_init(kEspUart, kEspBaudRate) != 0) {
        std::printf("[DIAG][ESP] UART%u init failed\n", kEspUart);
        idleForever();
    }

    constexpr const char* kCommands[] = {
        "AT\r\n",
        "ATE0\r\n",
        "AT+GMR\r\n",
        "AT+CWMODE?\r\n",
    };

    std::uint32_t command_index = 0;
    for (;;) {
        const char* command = kCommands[command_index % (sizeof(kCommands) / sizeof(kCommands[0]))];
        std::size_t length = 0;
        while (command[length] != '\0') {
            ++length;
        }
        const std::size_t written =
            amp_hal_uart_write_buffer(kEspUart, reinterpret_cast<const std::uint8_t*>(command), length);
        std::printf("[DIAG][ESP] tx=%s written=%u\n", command, static_cast<unsigned>(written));

        const std::uint64_t deadline = hal_micros() + 1000000ull;
        std::uint8_t rx[96]{};
        while (hal_micros() < deadline) {
            const std::size_t count = amp_hal_uart_read_buffer(kEspUart, rx, sizeof(rx) - 1);
            if (count > 0) {
                rx[count] = 0;
                std::printf("[DIAG][ESP] rx=%s\n", reinterpret_cast<const char*>(rx));
            }
            hal_delay_ms(20);
        }

        ++command_index;
        hal_delay_ms(1000);
    }
}

}  // extern "C"
