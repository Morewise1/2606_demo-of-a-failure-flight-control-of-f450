#include <memory>
#include <cstdio>

#include "Actuator.hpp"
#include "Controller.hpp"
#include "Estimator.hpp"
#include "FlightRtNode.hpp"
#include "Receiver.hpp"
#include "RtosCmdquIsrTransport.hpp"
#include "Sense.hpp"

int main() {
    amp::rt::I2cBusConfig i2c_config{};
    i2c_config.bus_id = 0;
    i2c_config.bus_speed_hz = 400000;
    i2c_config.mpu_addr = 0x68;
    i2c_config.mag_addr = 0x1E;
    i2c_config.use_magnetometer = true;

    std::unique_ptr<amp::rt::ImuBackend> imu_backend = std::make_unique<amp::rt::Mpu6050I2cBackend>(i2c_config);
    amp::rt::Hmc5883lConfig mag_config{};
    mag_config.bus_id = 0;
    mag_config.bus_speed_hz = 400000;
    mag_config.address = 0x1E;
    std::unique_ptr<amp::rt::MagBackend> mag_backend = std::make_unique<amp::rt::Hmc5883lI2cBackend>(mag_config);

    amp::rt::UartRxConfig uart_config{};
    uart_config.uart_id = 3;
    uart_config.baud_rate = 115200;

    std::unique_ptr<amp::rt::ReceiverBackend> rx_backend = std::make_unique<amp::rt::RtosUartBackend>(uart_config);
    amp::rt::Sense sense(std::move(imu_backend), std::move(mag_backend), false);
    amp::rt::Receiver receiver(std::move(rx_backend));
    amp::rt::Estimator estimator;
    amp::rt::Controller controller;
    amp::rt::PwmOutputConfig pwm_config{};
    pwm_config.channels = {7, 6, 10, 11};
    pwm_config.labels = {"PWM7", "PWM6", "PWM10", "PWM11"};
    amp::rt::Actuator actuator(std::make_unique<amp::rt::RtosPwmBackend>(pwm_config));
    std::unique_ptr<amp::rt::RtTransport> transport = std::make_unique<amp::rt::RtosCmdquIsrTransport>();

    amp::rt::FlightRtNode node(
        std::move(sense),
        std::move(receiver),
        std::move(estimator),
        std::move(controller),
        std::move(actuator),
        std::move(transport));

    const bool ok = node.run(0);
    std::printf("[RT] exit=%s\n", ok ? "ok" : "failed");
    return ok ? 0 : 1;
}
