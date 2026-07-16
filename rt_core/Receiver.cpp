#include "Receiver.hpp"

#include "RtosHal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace amp::rt {

namespace {

constexpr std::uint16_t kChannelCenterUs = 1500;
constexpr std::uint16_t kChannelMinUs = 1000;
constexpr std::uint16_t kChannelMaxUs = 2000;
constexpr std::size_t kRollChannel = 0;
constexpr std::size_t kPitchChannel = 1;
constexpr std::size_t kThrottleChannel = 2;
constexpr std::size_t kYawChannel = 3;
constexpr std::size_t kArmChannel = 4;
constexpr std::size_t kFailsafeChannel = 5;

std::uint16_t applyCenterDeadzone(std::uint16_t value_us, std::uint16_t deadzone_us) {
    const int delta = static_cast<int>(value_us) - static_cast<int>(kChannelCenterUs);
    if (std::abs(delta) < static_cast<int>(deadzone_us)) {
        return kChannelCenterUs;
    }
    return value_us;
}

std::uint16_t applyThrottleExpo(std::uint16_t value_us, float throttle_expo) {
    const float normalized =
        std::clamp((static_cast<float>(value_us) - static_cast<float>(kChannelMinUs)) /
                       static_cast<float>(kChannelMaxUs - kChannelMinUs),
                   0.0f,
                   1.0f);
    const float centered = (normalized - 0.5f) * 2.0f;
    const float shaped = ((1.0f - throttle_expo) * centered) + (throttle_expo * centered * centered * centered);
    const float output = (shaped * 0.5f) + 0.5f;
    const float pwm = static_cast<float>(kChannelMinUs) + output * static_cast<float>(kChannelMaxUs - kChannelMinUs);
    return static_cast<std::uint16_t>(std::clamp(std::lround(pwm), static_cast<long>(kChannelMinUs), static_cast<long>(kChannelMaxUs)));
}

std::array<std::uint8_t, kIbusFrameSize> encodeIbusFrame(const std::array<std::uint16_t, kIbusChannelCount>& channels) {
    std::array<std::uint8_t, kIbusFrameSize> frame{};
    frame[0] = static_cast<std::uint8_t>(kIbusFrameSize);
    frame[1] = 0x40;
    for (std::size_t channel = 0; channel < kIbusChannelCount; ++channel) {
        const std::uint16_t value = channels[channel];
        frame[2 + (channel * 2)] = static_cast<std::uint8_t>(value & 0xFFu);
        frame[3 + (channel * 2)] = static_cast<std::uint8_t>((value >> 8u) & 0xFFu);
    }
    std::uint16_t sum = 0;
    for (std::size_t index = 0; index < kIbusFrameSize - 2; ++index) {
        sum = static_cast<std::uint16_t>(sum + frame[index]);
    }
    const std::uint16_t crc = static_cast<std::uint16_t>(0xFFFFu - sum);
    frame[kIbusFrameSize - 2] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame[kIbusFrameSize - 1] = static_cast<std::uint8_t>((crc >> 8u) & 0xFFu);
    return frame;
}

}  // namespace

bool SimulatedIbusBackend::init() {
    pending_.clear();
    tick_ = 0;
    return true;
}

std::size_t SimulatedIbusBackend::read(std::uint8_t* buffer, std::size_t max_bytes) {
    if (buffer == nullptr || max_bytes == 0) {
        return 0;
    }
    ++tick_;

    if ((tick_ % frame_period_us_) == 0u || pending_.empty()) {
        const float phase = static_cast<float>(tick_) * 0.0007f;
        channels_[0] = static_cast<std::uint16_t>(1500 + 200 * std::sin(phase));
        channels_[1] = static_cast<std::uint16_t>(1500 + 180 * std::sin(phase * 0.8f));
        channels_[2] = 1320;
        channels_[3] = static_cast<std::uint16_t>(1500 + 100 * std::sin(phase * 0.6f));
        channels_[5] = (tick_ > 1600u && tick_ < 1700u) ? 1700 : 1200;
        auto frame = encodeIbusFrame(channels_);
        pending_.insert(pending_.end(), frame.begin(), frame.end());
    }

    const std::size_t count = std::min(max_bytes, pending_.size());
    std::memcpy(buffer, pending_.data(), count);
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(count));
    return count;
}

Receiver::Receiver(std::unique_ptr<ReceiverBackend> backend, std::uint16_t arm_switch_threshold_us)
    : backend_(std::move(backend)), arm_switch_threshold_us_(arm_switch_threshold_us) {}

bool Receiver::init() {
    frame_index_ = 0;
    last_frame_us_ = 0;
    last_sample_ = ReceiverSample{};
    return backend_ && backend_->init();
}

bool Receiver::poll(ReceiverSample& sample) {
    if (!backend_) {
        return false;
    }

    std::uint8_t raw[96]{};
    const std::size_t count = backend_->read(raw, sizeof(raw));
    bool frame_updated = false;
    for (std::size_t index = 0; index < count; ++index) {
        if (pushByte(raw[index], sample)) {
            frame_updated = true;
            sample = last_sample_;
        }
    }
    if (!frame_updated) {
        sample = last_sample_;
        sample.link_ok = hasLink(monotonicNowUs(), 80000);
        sample.failsafe_switch = sample.channels[kFailsafeChannel] > arm_switch_threshold_us_;
        if (!sample.link_ok) {
            sample.channels[kThrottleChannel] = kChannelMinUs;
            sample.channels[kArmChannel] = kChannelMinUs;
            sample.channels[kRollChannel] = kChannelCenterUs;
            sample.channels[kPitchChannel] = kChannelCenterUs;
            sample.channels[kYawChannel] = kChannelCenterUs;
            sample.failsafe_switch = true;
            sample.link_ok = false;
            last_sample_ = sample;
        }
    }
    return true;
}

bool Receiver::hasLink(std::uint64_t now_us, std::uint64_t timeout_us) const {
    if (last_frame_us_ == 0) {
        return false;
    }
    return (now_us - last_frame_us_) <= timeout_us;
}

const ReceiverSample& Receiver::last() const {
    return last_sample_;
}

void Receiver::applyConfig(const proto::FlightConfig& config) {
    deadzone_us_ = std::clamp<std::uint16_t>(config.deadzone_us, 0u, 100u);
    throttle_expo_ = std::clamp(config.throttle_expo, 0.0f, 0.95f);
}

bool Receiver::pushByte(std::uint8_t byte, ReceiverSample& sample) {
    if (frame_index_ == 0) {
        if (byte != static_cast<std::uint8_t>(kIbusFrameSize)) {
            return false;
        }
        frame_buffer_[frame_index_++] = byte;
        return false;
    }

    if (frame_index_ == 1 && byte != 0x40u) {
        frame_index_ = 0;
        return false;
    }

    frame_buffer_[frame_index_++] = byte;
    if (frame_index_ < kIbusFrameSize) {
        return false;
    }

    frame_index_ = 0;
    if (!decodeFrame(frame_buffer_, sample)) {
        return false;
    }

    last_frame_us_ = monotonicNowUs();
    sample.timestamp_us = last_frame_us_;
    sample.link_ok = true;
    sample.failsafe_switch = sample.channels[kFailsafeChannel] > arm_switch_threshold_us_;
    last_sample_ = sample;
    return true;
}

bool Receiver::decodeFrame(const std::array<std::uint8_t, kIbusFrameSize>& frame, ReceiverSample& sample) const {
    const std::uint16_t expected = checksum(frame);
    const std::uint16_t received =
        static_cast<std::uint16_t>(frame[kIbusFrameSize - 2] | (static_cast<std::uint16_t>(frame[kIbusFrameSize - 1]) << 8u));
    if (expected != received) {
        return false;
    }
    for (std::size_t channel = 0; channel < kIbusChannelCount; ++channel) {
        const std::size_t offset = 2 + channel * 2;
        sample.channels[channel] = static_cast<std::uint16_t>(frame[offset] | (static_cast<std::uint16_t>(frame[offset + 1]) << 8u));
    }
    sample.channels[kRollChannel] = applyCenterDeadzone(sample.channels[kRollChannel], deadzone_us_);
    sample.channels[kPitchChannel] = applyCenterDeadzone(sample.channels[kPitchChannel], deadzone_us_);
    sample.channels[kYawChannel] = applyCenterDeadzone(sample.channels[kYawChannel], deadzone_us_);
    sample.channels[kThrottleChannel] = applyThrottleExpo(sample.channels[kThrottleChannel], throttle_expo_);
    return true;
}

std::uint16_t Receiver::checksum(const std::array<std::uint8_t, kIbusFrameSize>& frame) {
    std::uint16_t sum = 0;
    for (std::size_t index = 0; index < kIbusFrameSize - 2; ++index) {
        sum = static_cast<std::uint16_t>(sum + frame[index]);
    }
    return static_cast<std::uint16_t>(0xFFFFu - sum);
}

std::uint64_t Receiver::monotonicNowUs() {
    return hal_micros();
}

}  // namespace amp::rt
