#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "../common/protocol.hpp"

namespace amp::rt {

constexpr std::size_t kIbusFrameSize = 32;
constexpr std::size_t kIbusChannelCount = 14;

struct ReceiverSample {
    std::array<std::uint16_t, kIbusChannelCount> channels{};
    bool link_ok{false};
    bool failsafe_switch{false};
    std::uint64_t timestamp_us{0};
};

class ReceiverBackend {
public:
    virtual ~ReceiverBackend() = default;
    virtual bool init() = 0;
    virtual std::size_t read(std::uint8_t* buffer, std::size_t max_bytes) = 0;
};

class SimulatedIbusBackend final : public ReceiverBackend {
public:
    bool init() override;
    std::size_t read(std::uint8_t* buffer, std::size_t max_bytes) override;

private:
    std::array<std::uint16_t, kIbusChannelCount> channels_{
        1500, 1500, 1200, 1500, 1700, 1200, 1000,
        1000, 1000, 1000, 1000, 1000, 1000, 1000};
    std::vector<std::uint8_t> pending_;
    std::uint64_t tick_{0};
    std::uint64_t frame_period_us_{7000};
};

struct UartRxConfig {
    std::uint8_t uart_id{3};
    std::uint32_t baud_rate{115200};
};

class RtosUartBackend final : public ReceiverBackend {
public:
    explicit RtosUartBackend(UartRxConfig config = {});
    ~RtosUartBackend() override = default;

    bool init() override;
    std::size_t read(std::uint8_t* buffer, std::size_t max_bytes) override;

private:
    UartRxConfig config_;
};

class Receiver {
public:
    explicit Receiver(std::unique_ptr<ReceiverBackend> backend, std::uint16_t arm_switch_threshold_us = 1500);
    bool init();
    bool poll(ReceiverSample& sample);
    bool hasLink(std::uint64_t now_us, std::uint64_t timeout_us) const;
    const ReceiverSample& last() const;
    void applyConfig(const proto::FlightConfig& config);

private:
    bool pushByte(std::uint8_t byte, ReceiverSample& sample);
    bool decodeFrame(const std::array<std::uint8_t, kIbusFrameSize>& frame, ReceiverSample& sample) const;
    static std::uint16_t checksum(const std::array<std::uint8_t, kIbusFrameSize>& frame);
    static std::uint64_t monotonicNowUs();

    std::unique_ptr<ReceiverBackend> backend_;
    std::array<std::uint8_t, kIbusFrameSize> frame_buffer_{};
    std::size_t frame_index_{0};
    ReceiverSample last_sample_{};
    std::uint64_t last_frame_us_{0};
    std::uint16_t arm_switch_threshold_us_{1500};
    std::uint16_t deadzone_us_{15};
    float throttle_expo_{0.45f};
};

}  // namespace amp::rt
