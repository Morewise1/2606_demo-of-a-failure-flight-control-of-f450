#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

std::uint64_t hal_micros();
std::uint64_t hal_time_us();
void hal_delay_ms(std::uint32_t delay_ms);
std::uint32_t hal_rtos_tick_count();
std::uint32_t hal_rtos_us_to_ticks(std::uint32_t duration_us);
void hal_rtos_delay_until(std::uint32_t* previous_wake_tick, std::uint32_t period_ticks);

int hal_i2c_init(std::uint8_t bus_id, std::uint32_t speed_hz);
int hal_i2c_write(std::uint8_t bus_id, std::uint8_t device_addr, const std::uint8_t* data, std::size_t length);
int hal_i2c_read(std::uint8_t bus_id, std::uint8_t device_addr, std::uint8_t* data, std::size_t length);

int hal_pwm_init(std::uint32_t channel, std::uint32_t period_ns);
int hal_pwm_set_duty(std::uint32_t channel, std::uint32_t high_time_ns);
int hal_pwm_enable(std::uint32_t channel, bool enable);

int amp_hal_uart_init(std::uint8_t uart_id, std::uint32_t baud_rate);
std::size_t amp_hal_uart_read_buffer(std::uint8_t uart_id, std::uint8_t* buffer, std::size_t max_bytes);
std::size_t amp_hal_uart_write_buffer(std::uint8_t uart_id, const std::uint8_t* buffer, std::size_t length);

}
