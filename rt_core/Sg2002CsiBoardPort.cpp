#include "RtosHal.hpp"

#if defined(AMP_USE_SG2002_CSI_PORT)

extern "C" {
#if defined(__has_include)
#if __has_include("cvitek/driver/i2c/include/i2c.h")
#include "cvitek/driver/i2c/include/i2c.h"
#elif __has_include("i2c.h")
#include "i2c.h"
#else
void i2c_init(unsigned char i2c_id);
int i2c_write(unsigned char i2c_id, unsigned char dev, unsigned short addr, unsigned short alen, unsigned char* buffer, unsigned short len);
int i2c_read(unsigned char i2c_id, unsigned char dev, unsigned short addr, unsigned short alen, unsigned char* buffer, unsigned short len);
#endif

#if __has_include("delay.h")
#include "delay.h"
#else
void udelay(unsigned int us);
#endif

#if __has_include("core_rv64.h")
#include "core_rv64.h"
#else
unsigned long long csi_coret_get_value();
#endif

#if __has_include("pwm.h")
#include "pwm.h"
#else
typedef struct {
    unsigned char pwm_id;
    unsigned int channel;
    unsigned int period;
    unsigned int pulse;
} pwm_configuration_t;
void pwm_init(pwm_configuration_t* cfg);
void pwm_set_output_cfg(pwm_configuration_t* cfg);
#endif
#endif

// SDK UART function name is also hal_uart_init(). Our project-level wrapper is
// amp_hal_uart_init(), so this board port can safely call the SDK C symbol.
typedef int device_uart;
int hal_uart_init(device_uart dev_uart, int baudrate, int uart_clock);
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {

constexpr std::size_t kMaxI2cBus = 4;
constexpr std::size_t kMaxI2cAddr = 128;
constexpr std::size_t kMaxPwmChannel = 16;
constexpr std::size_t kPwmChannelsPerBank = 4;
constexpr std::uint32_t kDefaultUartClockHz = 25000000;
constexpr std::uintptr_t kUart0Base = 0x04140000u;
constexpr std::uintptr_t kUart1Base = 0x04150000u;
constexpr std::uintptr_t kUart2Base = 0x04160000u;
constexpr std::uintptr_t kUart3Base = 0x04170000u;
constexpr std::uintptr_t kUartRbrThrOffset = 0x00u;
constexpr std::uintptr_t kUartLsrOffset = 0x14u;
constexpr std::uint32_t kUartLsrDataReady = 1u << 0u;
constexpr std::uint32_t kUartLsrTxEmpty = 1u << 5u;
constexpr std::uint64_t kCoreTimerHz =
#if defined(AMP_CORET_HZ)
    static_cast<std::uint64_t>(AMP_CORET_HZ);
#elif defined(configCPU_CLOCK_HZ)
    static_cast<std::uint64_t>(configCPU_CLOCK_HZ);
#else
    25000000ull;
#endif
constexpr std::uint64_t kPwmClockHz =
#if defined(AMP_PWM_CLOCK_HZ)
    static_cast<std::uint64_t>(AMP_PWM_CLOCK_HZ);
#else
    100000000ull;
#endif

struct I2cCursor {
    std::uint16_t reg{0};
    bool valid{false};
};

std::array<bool, kMaxI2cBus> g_i2c_ready{};
std::array<std::array<I2cCursor, kMaxI2cAddr>, kMaxI2cBus> g_i2c_cursor{};
std::array<std::uint32_t, kMaxPwmChannel> g_pwm_period_ns{};
std::array<bool, kMaxPwmChannel> g_pwm_ready{};

std::uint32_t nsToPwmTicks(std::uint32_t duration_ns) {
    const std::uint64_t ticks = (static_cast<std::uint64_t>(duration_ns) * kPwmClockHz + 999999999ull) / 1000000000ull;
    return static_cast<std::uint32_t>(ticks == 0 ? 1 : ticks);
}

bool rememberI2cRegister(std::uint8_t bus_id, std::uint8_t device_addr, std::uint16_t reg) {
    if (bus_id >= g_i2c_cursor.size() || device_addr >= kMaxI2cAddr) {
        return false;
    }
    g_i2c_cursor[bus_id][device_addr].reg = reg;
    g_i2c_cursor[bus_id][device_addr].valid = true;
    return true;
}

bool getI2cRegister(std::uint8_t bus_id, std::uint8_t device_addr, std::uint16_t& reg) {
    if (bus_id >= g_i2c_cursor.size() || device_addr >= kMaxI2cAddr) {
        return false;
    }
    const I2cCursor& cursor = g_i2c_cursor[bus_id][device_addr];
    if (!cursor.valid) {
        return false;
    }
    reg = cursor.reg;
    return true;
}

std::uintptr_t uartBase(std::uint8_t uart_id) {
    switch (uart_id) {
        case 0:
            return kUart0Base;
        case 1:
            return kUart1Base;
        case 2:
            return kUart2Base;
        case 3:
            return kUart3Base;
        default:
            return 0;
    }
}

inline std::uint32_t mmioRead32(std::uintptr_t address) {
    return *reinterpret_cast<volatile std::uint32_t*>(address);
}

inline void mmioWrite32(std::uintptr_t address, std::uint32_t value) {
    *reinterpret_cast<volatile std::uint32_t*>(address) = value;
}

inline std::uint32_t uartReadLsr(std::uintptr_t base) {
    return mmioRead32(base + kUartLsrOffset);
}

inline std::uint8_t uartReadRbr(std::uintptr_t base) {
    return static_cast<std::uint8_t>(mmioRead32(base + kUartRbrThrOffset) & 0xFFu);
}

inline void uartWriteThr(std::uintptr_t base, std::uint8_t ch) {
    mmioWrite32(base + kUartRbrThrOffset, ch);
}

bool waitUartTxReady(std::uintptr_t base) {
    constexpr std::uint32_t kSpinLimit = 200000;
    for (std::uint32_t spin = 0; spin < kSpinLimit; ++spin) {
        if ((uartReadLsr(base) & kUartLsrTxEmpty) != 0u) {
            return true;
        }
    }
    return false;
}

}  // namespace

extern "C" {

int hal_board_i2c_init(std::uint8_t bus_id, std::uint32_t speed_hz) {
    (void)speed_hz;
    if (bus_id >= g_i2c_ready.size()) {
        return -1;
    }
    if (!g_i2c_ready[bus_id]) {
        i2c_init(bus_id);
        g_i2c_ready[bus_id] = true;
    }
    return 0;
}

int hal_board_i2c_write(std::uint8_t bus_id, std::uint8_t device_addr, const std::uint8_t* data, std::size_t length) {
    if (bus_id >= g_i2c_ready.size() || !g_i2c_ready[bus_id] || data == nullptr || length == 0) {
        return -1;
    }

    if (length == 1) {
        return rememberI2cRegister(bus_id, device_addr, data[0]) ? 0 : -1;
    }

    const std::uint16_t reg = data[0];
    auto* payload = const_cast<std::uint8_t*>(data + 1);
    const auto payload_len = static_cast<std::uint16_t>(length - 1);
    const int ret = i2c_write(bus_id, device_addr, reg, 1, payload, payload_len);
    if (ret == 0) {
        rememberI2cRegister(bus_id, device_addr, static_cast<std::uint16_t>(reg + payload_len));
    }
    return ret == 0 ? 0 : -1;
}

int hal_board_i2c_read(std::uint8_t bus_id, std::uint8_t device_addr, std::uint8_t* data, std::size_t length) {
    if (bus_id >= g_i2c_ready.size() || !g_i2c_ready[bus_id] || data == nullptr || length == 0) {
        return -1;
    }

    std::uint16_t reg = 0;
    if (!getI2cRegister(bus_id, device_addr, reg)) {
        return -1;
    }

    const int ret = i2c_read(bus_id, device_addr, reg, 1, data, static_cast<std::uint16_t>(length));
    if (ret == 0) {
        rememberI2cRegister(bus_id, device_addr, static_cast<std::uint16_t>(reg + length));
    }
    return ret == 0 ? 0 : -1;
}

std::uint64_t hal_board_micros() {
    const std::uint64_t ticks = csi_coret_get_value();
    return (ticks * 1000000ull) / kCoreTimerHz;
}

int hal_board_pwm_init(std::uint32_t channel, std::uint32_t period_ns) {
    if (channel >= g_pwm_ready.size()) {
        return -1;
    }
    g_pwm_period_ns[channel] = period_ns;
    g_pwm_ready[channel] = true;

    pwm_configuration_t cfg{};
    cfg.pwm_id = static_cast<std::uint8_t>(channel / kPwmChannelsPerBank);
    cfg.channel = static_cast<std::uint32_t>(channel % kPwmChannelsPerBank);
    cfg.period = nsToPwmTicks(period_ns);
    cfg.pulse = nsToPwmTicks(1000000u);
    pwm_init(&cfg);
    return 0;
}

int hal_board_pwm_set_duty(std::uint32_t channel, std::uint32_t high_time_ns) {
    if (channel >= g_pwm_ready.size() || !g_pwm_ready[channel]) {
        return -1;
    }
    pwm_configuration_t cfg{};
    cfg.pwm_id = static_cast<std::uint8_t>(channel / kPwmChannelsPerBank);
    cfg.channel = static_cast<std::uint32_t>(channel % kPwmChannelsPerBank);
    cfg.period = nsToPwmTicks(g_pwm_period_ns[channel]);
    cfg.pulse = nsToPwmTicks(high_time_ns);
    pwm_set_output_cfg(&cfg);
    return 0;
}

int hal_board_pwm_enable(std::uint32_t channel, bool enable) {
    if (channel >= g_pwm_ready.size() || !g_pwm_ready[channel]) {
        return -1;
    }
    if (!enable) {
        pwm_configuration_t cfg{};
        cfg.pwm_id = static_cast<std::uint8_t>(channel / kPwmChannelsPerBank);
        cfg.channel = static_cast<std::uint32_t>(channel % kPwmChannelsPerBank);
        cfg.period = nsToPwmTicks(g_pwm_period_ns[channel]);
        cfg.pulse = 0;
        pwm_set_output_cfg(&cfg);
    }
    return 0;
}

int hal_board_uart_init(std::uint8_t uart_id, std::uint32_t baud_rate) {
    if (uartBase(uart_id) == 0) {
        return -1;
    }
    hal_uart_init(static_cast<device_uart>(uart_id), static_cast<int>(baud_rate), static_cast<int>(kDefaultUartClockHz));
    return 0;
}

std::size_t hal_board_uart_read_buffer(std::uint8_t uart_id, std::uint8_t* buffer, std::size_t max_bytes) {
    if (buffer == nullptr || max_bytes == 0) {
        return 0;
    }
    const std::uintptr_t base = uartBase(uart_id);
    if (base == 0) {
        return 0;
    }
    std::size_t count = 0;
    while (count < max_bytes && ((uartReadLsr(base) & kUartLsrDataReady) != 0u)) {
        buffer[count++] = uartReadRbr(base);
    }
    return count;
}

std::size_t hal_board_uart_write_buffer(std::uint8_t uart_id, const std::uint8_t* buffer, std::size_t length) {
    if (buffer == nullptr || length == 0) {
        return 0;
    }
    const std::uintptr_t base = uartBase(uart_id);
    if (base == 0) {
        return 0;
    }
    for (std::size_t index = 0; index < length; ++index) {
        if (!waitUartTxReady(base)) {
            return index;
        }
        uartWriteThr(base, buffer[index]);
    }
    return length;
}

}  // extern "C"

#endif  // AMP_USE_SG2002_CSI_PORT
