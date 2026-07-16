#include "Receiver.hpp"

#include "RtosHal.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace amp::rt {

RtosUartBackend::RtosUartBackend(UartRxConfig config) : config_(std::move(config)) {}

bool RtosUartBackend::init() {
    if (amp_hal_uart_init(config_.uart_id, config_.baud_rate) != 0) {
        std::printf("[RT][UART] UART%u init failed at %lu baud\n",
                    config_.uart_id,
                    static_cast<unsigned long>(config_.baud_rate));
        return false;
    }
    std::printf("[RT][UART] UART%u ready at %lu baud\n",
                config_.uart_id,
                static_cast<unsigned long>(config_.baud_rate));
    return true;
}

std::size_t RtosUartBackend::read(std::uint8_t* buffer, std::size_t max_bytes) {
    if (buffer == nullptr || max_bytes == 0) {
        return 0;
    }

    const std::size_t count = amp_hal_uart_read_buffer(config_.uart_id, buffer, max_bytes);
    return std::min(count, max_bytes);
}

}  // namespace amp::rt
