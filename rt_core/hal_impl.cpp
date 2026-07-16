#include "RtosHal.hpp"

#include "Actuator.hpp"
#include "Controller.hpp"
#include "Estimator.hpp"
#include "FlightRtNode.hpp"
#include "Receiver.hpp"
#include "RtosCmdquIsrTransport.hpp"
#include "Sense.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <utility>

#if defined(__has_include)
#if __has_include("FreeRTOS.h") && __has_include("task.h")
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}
#define AMP_FC_HAS_FREERTOS_HEADERS 1
#endif
#endif

#ifndef AMP_FC_HAS_FREERTOS_HEADERS
#define AMP_FC_HAS_FREERTOS_HEADERS 0
#endif

namespace {

constexpr std::uint8_t kI2cBusMpu6050 = 0;
constexpr std::uint8_t kMpu6050Address = 0x68;
constexpr std::uint8_t kMagnetometerAddress = 0x1E;
constexpr std::uint8_t kIbusUart = 3;
constexpr std::uint32_t kI2cSpeedHz = 400000;
constexpr std::uint32_t kIbusBaudRate = 115200;
constexpr std::uint32_t kDefaultTickRateHz = 1000;

#if AMP_FC_HAS_FREERTOS_HEADERS
using RtosTick = TickType_t;

std::uint32_t tickRateHz() {
    return static_cast<std::uint32_t>(configTICK_RATE_HZ);
}
#else
using RtosTick = std::uint32_t;

std::uint32_t tickRateHz() {
    return kDefaultTickRateHz;
}
#endif

std::uint32_t usToTicksCeil(std::uint32_t duration_us) {
    const std::uint64_t ticks =
        (static_cast<std::uint64_t>(duration_us) * tickRateHz() + 999999ull) / 1000000ull;
    return static_cast<std::uint32_t>(std::max<std::uint64_t>(ticks, 1ull));
}

}  // namespace

extern "C" {

// TODO: SDK API. Replace these weak board-port stubs with real SG2002/XuanTie CSI bindings:
// csi_iic_init / csi_iic_master_send / csi_iic_master_receive,
// csi_uart_init / csi_uart_receive, and CVI PWM register or SDK calls.
#if defined(__GNUC__)
#define AMP_FC_WEAK __attribute__((weak))
#else
#define AMP_FC_WEAK
#endif

AMP_FC_WEAK int hal_board_i2c_init(std::uint8_t bus_id, std::uint32_t speed_hz) {
    (void)bus_id;
    (void)speed_hz;
    return -1;
}

AMP_FC_WEAK int hal_board_i2c_write(
    std::uint8_t bus_id,
    std::uint8_t device_addr,
    const std::uint8_t* data,
    std::size_t length) {
    (void)bus_id;
    (void)device_addr;
    (void)data;
    (void)length;
    return -1;
}

AMP_FC_WEAK int hal_board_i2c_read(std::uint8_t bus_id, std::uint8_t device_addr, std::uint8_t* data, std::size_t length) {
    (void)bus_id;
    (void)device_addr;
    (void)data;
    (void)length;
    return -1;
}

AMP_FC_WEAK int hal_board_pwm_init(std::uint32_t channel, std::uint32_t period_ns) {
    (void)channel;
    (void)period_ns;
    return -1;
}

AMP_FC_WEAK int hal_board_pwm_set_duty(std::uint32_t channel, std::uint32_t high_time_ns) {
    (void)channel;
    (void)high_time_ns;
    return -1;
}

AMP_FC_WEAK int hal_board_pwm_enable(std::uint32_t channel, bool enable) {
    (void)channel;
    (void)enable;
    return -1;
}

AMP_FC_WEAK int hal_board_uart_init(std::uint8_t uart_id, std::uint32_t baud_rate) {
    (void)uart_id;
    (void)baud_rate;
    return -1;
}

AMP_FC_WEAK std::size_t hal_board_uart_read_buffer(std::uint8_t uart_id, std::uint8_t* buffer, std::size_t max_bytes) {
    (void)uart_id;
    (void)buffer;
    (void)max_bytes;
    return 0;
}

AMP_FC_WEAK std::size_t hal_board_uart_write_buffer(std::uint8_t uart_id, const std::uint8_t* buffer, std::size_t length) {
    (void)uart_id;
    (void)buffer;
    (void)length;
    return 0;
}

AMP_FC_WEAK std::uint64_t hal_board_micros() {
    return 0;
}

#if !AMP_FC_HAS_FREERTOS_HEADERS
AMP_FC_WEAK std::uint32_t hal_board_rtos_tick_count() {
    return 0;
}

AMP_FC_WEAK void hal_board_rtos_delay_until(std::uint32_t* previous_wake_tick, std::uint32_t period_ticks) {
    if (previous_wake_tick != nullptr) {
        *previous_wake_tick += period_ticks;
    }
}
#endif

std::uint64_t hal_micros() {
    // TODO: SDK API. Prefer a hardware microsecond timer, e.g. csi_tick_get_us().
    return hal_board_micros();
}

std::uint64_t hal_time_us() {
    return hal_micros();
}

void hal_delay_ms(std::uint32_t delay_ms) {
#if AMP_FC_HAS_FREERTOS_HEADERS
    vTaskDelay(static_cast<RtosTick>(usToTicksCeil(delay_ms * 1000u)));
#else
    std::uint32_t wake_tick = hal_board_rtos_tick_count();
    hal_board_rtos_delay_until(&wake_tick, usToTicksCeil(delay_ms * 1000u));
#endif
}

std::uint32_t hal_rtos_tick_count() {
#if AMP_FC_HAS_FREERTOS_HEADERS
    return static_cast<std::uint32_t>(xTaskGetTickCount());
#else
    return hal_board_rtos_tick_count();
#endif
}

std::uint32_t hal_rtos_us_to_ticks(std::uint32_t duration_us) {
    return usToTicksCeil(duration_us);
}

void hal_rtos_delay_until(std::uint32_t* previous_wake_tick, std::uint32_t period_ticks) {
    if (previous_wake_tick == nullptr || period_ticks == 0u) {
        return;
    }

#if AMP_FC_HAS_FREERTOS_HEADERS
    RtosTick wake_tick = static_cast<RtosTick>(*previous_wake_tick);
    vTaskDelayUntil(&wake_tick, static_cast<RtosTick>(period_ticks));
    *previous_wake_tick = static_cast<std::uint32_t>(wake_tick);
#else
    hal_board_rtos_delay_until(previous_wake_tick, period_ticks);
#endif
}

int hal_i2c_init(std::uint8_t bus_id, std::uint32_t speed_hz) {
    // TODO: SDK API. Typical mapping: csi_iic_init(bus_id), csi_iic_speed(bus_id, speed_hz).
    return hal_board_i2c_init(bus_id, speed_hz);
}

int hal_i2c_write(std::uint8_t bus_id, std::uint8_t device_addr, const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return -1;
    }
    // TODO: SDK API. Typical mapping: csi_iic_master_send(bus_id, device_addr, data, length, timeout_ms).
    return hal_board_i2c_write(bus_id, device_addr, data, length);
}

int hal_i2c_read(std::uint8_t bus_id, std::uint8_t device_addr, std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return -1;
    }
    // TODO: SDK API. Typical mapping: csi_iic_master_receive(bus_id, device_addr, data, length, timeout_ms).
    return hal_board_i2c_read(bus_id, device_addr, data, length);
}

int hal_pwm_init(std::uint32_t channel, std::uint32_t period_ns) {
    // TODO: SDK API. Typical mapping: cvi_pwm_init(channel), cvi_pwm_set_period(channel, period_ns).
    return hal_board_pwm_init(channel, period_ns);
}

int hal_pwm_set_duty(std::uint32_t channel, std::uint32_t high_time_ns) {
    // TODO: SDK API. Typical mapping: cvi_pwm_set_duty(channel, high_time_ns).
    return hal_board_pwm_set_duty(channel, high_time_ns);
}

int hal_pwm_enable(std::uint32_t channel, bool enable) {
    // TODO: SDK API. Typical mapping: cvi_pwm_enable(channel, enable).
    return hal_board_pwm_enable(channel, enable);
}

int amp_hal_uart_init(std::uint8_t uart_id, std::uint32_t baud_rate) {
    // TODO: SDK API. Typical mapping: csi_uart_init(uart_id), csi_uart_baud(uart_id, baud_rate), enable RX IRQ/FIFO.
    return hal_board_uart_init(uart_id, baud_rate);
}

std::size_t amp_hal_uart_read_buffer(std::uint8_t uart_id, std::uint8_t* buffer, std::size_t max_bytes) {
    if (buffer == nullptr || max_bytes == 0) {
        return 0;
    }
    // TODO: SDK API. Typical non-blocking mapping: csi_uart_receive(uart_id, buffer, max_bytes, timeout=0).
    return hal_board_uart_read_buffer(uart_id, buffer, max_bytes);
}

std::size_t amp_hal_uart_write_buffer(std::uint8_t uart_id, const std::uint8_t* buffer, std::size_t length) {
    if (buffer == nullptr || length == 0) {
        return 0;
    }
    return hal_board_uart_write_buffer(uart_id, buffer, length);
}

void amp_fc_on_cmdqu_message(unsigned char cmd_id, unsigned int seq_hint) {
    amp::rt::RtosCmdquIsrTransport::onCmdFromMailbox(
        static_cast<std::uint8_t>(cmd_id),
        static_cast<std::uint32_t>(seq_hint));
}

int amp_fc_mailbox_isr(int irq, void* context) {
    return amp::rt::RtosCmdquIsrTransport::mailboxIsr(irq, context);
}

void flight_control_task(void* pvParameters) {
    (void)pvParameters;

    amp::rt::I2cBusConfig i2c_config{};
    i2c_config.bus_id = kI2cBusMpu6050;
    i2c_config.bus_speed_hz = kI2cSpeedHz;
    i2c_config.mpu_addr = kMpu6050Address;
    i2c_config.mag_addr = kMagnetometerAddress;
    i2c_config.use_magnetometer = true;

    amp::rt::UartRxConfig uart_config{};
    uart_config.uart_id = kIbusUart;
    uart_config.baud_rate = kIbusBaudRate;

    amp::rt::Hmc5883lConfig mag_config{};
    mag_config.bus_id = kI2cBusMpu6050;
    mag_config.bus_speed_hz = kI2cSpeedHz;
    mag_config.address = kMagnetometerAddress;

    auto imu_backend = std::make_unique<amp::rt::Mpu6050I2cBackend>(i2c_config);
    auto mag_backend = std::make_unique<amp::rt::Hmc5883lI2cBackend>(mag_config);
    auto rx_backend = std::make_unique<amp::rt::RtosUartBackend>(uart_config);

    amp::rt::Sense sense(std::move(imu_backend), std::move(mag_backend), false);
    amp::rt::Receiver receiver(std::move(rx_backend));
    amp::rt::Estimator estimator;
    amp::rt::Controller controller;

    amp::rt::PwmOutputConfig pwm_config{};
    pwm_config.channels = {7, 6, 10, 11};
    pwm_config.labels = {"PWM7", "PWM6", "PWM10", "PWM11"};
    amp::rt::Actuator actuator(std::make_unique<amp::rt::RtosPwmBackend>(pwm_config));

    std::unique_ptr<amp::rt::RtTransport> transport = std::make_unique<amp::rt::RtosCmdquIsrTransport>();
    amp::rt::FlightRtNode flight_node(
        std::move(sense),
        std::move(receiver),
        std::move(estimator),
        std::move(controller),
        std::move(actuator),
        std::move(transport));

    // FlightRtNode::run(0) means run forever; the internal scheduler is 1 kHz.
    // Calling run(1000) with the current API would stop after about one second.
    const bool ok = flight_node.run(0);
    std::printf("[RT][TASK] flight_control_task exited: %s\n", ok ? "ok" : "failed");

    for (;;) {
        hal_delay_ms(1000);
    }
}

#undef AMP_FC_WEAK

}  // extern "C"
