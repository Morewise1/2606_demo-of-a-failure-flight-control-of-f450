#include "FlightRtNode.hpp"

#include "RtosHal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace amp::rt {

namespace {

float mapChannelUs(std::uint16_t pulse_us, float output_min, float output_max) {
    const float input = std::clamp(static_cast<float>(pulse_us), 1000.0f, 2000.0f);
    const float ratio = (input - 1000.0f) / 1000.0f;
    return output_min + ratio * (output_max - output_min);
}

}  // namespace

bool LoopbackRtTransport::init() {
    tick_ = 0;
    publish_count_ = 0;
    return true;
}

bool LoopbackRtTransport::poll(IncomingCommand& command) {
    command = IncomingCommand{};
    ++tick_;

    if (tick_ == 40) {
        proto::CommandArm arm{};
        arm.state = static_cast<std::uint8_t>(proto::ArmState::Armed);
        command.arm = arm;
    }
    if ((tick_ % 10u) == 0u) {
        proto::CommandSetpoint setpoint{};
        setpoint.throttle = 0.32f;
        setpoint.roll_deg = 6.0f * std::sin(static_cast<float>(tick_) * 0.01f);
        setpoint.pitch_deg = 5.0f * std::sin(static_cast<float>(tick_) * 0.008f);
        setpoint.yaw_rate_dps = 10.0f * std::sin(static_cast<float>(tick_) * 0.012f);
        setpoint.seq = static_cast<std::uint32_t>(tick_);
        command.setpoint = setpoint;
    }
    if ((tick_ % 20u) == 0u) {
        proto::Heartbeat heartbeat{};
        heartbeat.monotonic_us = tick_ * 1000u;
        heartbeat.seq = static_cast<std::uint32_t>(tick_);
        command.heartbeat = heartbeat;
    }
    if (tick_ == 600u) {
        auto config = proto::default_flight_config();
        config.angle_p = 5.5f;
        config.rate_p = 0.13f;
        config.rate_i = 0.02f;
        config.rate_d = 0.003f;
        config.throttle_expo = 0.45f;
        config.deadzone_us = 15u;
        config.update_seq = static_cast<std::uint32_t>(tick_);
        command.config = config;
    }
    if (tick_ == 2400) {
        command.emergency_stop = true;
    }
    return true;
}

bool LoopbackRtTransport::publish(const proto::RtReport& report) {
    ++publish_count_;
    if ((publish_count_ % 120u) == 0u) {
        std::printf("[RT] att=%.2f,%.2f,%.2f pwm=%u,%u,%u,%u flags=0x%08lX\n",
                    report.roll_deg,
                    report.pitch_deg,
                    report.yaw_deg,
                    report.motor_pwm_us[0],
                    report.motor_pwm_us[1],
                    report.motor_pwm_us[2],
                    report.motor_pwm_us[3],
                    static_cast<unsigned long>(report.status_flags));
    }
    return true;
}

FlightRtNode::FlightRtNode(
    Sense sense,
    Receiver receiver,
    Estimator estimator,
    Controller controller,
    Actuator actuator,
    std::unique_ptr<RtTransport> transport)
    : sense_(std::move(sense)),
      receiver_(std::move(receiver)),
      estimator_(std::move(estimator)),
      controller_(std::move(controller)),
      actuator_(std::move(actuator)),
      transport_(std::move(transport)) {}

bool FlightRtNode::init() {
    if (!sense_.init()) {
        return false;
    }
    if (!receiver_.init()) {
        return false;
    }
    estimator_.setGyroBias(sense_.gyro_bias_dps());
    if (!actuator_.init()) {
        return false;
    }
    if (!transport_ || !transport_->init()) {
        return false;
    }
    controller_.setArmState(proto::ArmState::Disarmed);
    const auto boot_config = proto::default_flight_config();
    controller_.applyConfig(boot_config);
    receiver_.applyConfig(boot_config);
    last_setpoint_us_ = monotonicNowUs();
    last_heartbeat_us_ = 0;
    report_seq_ = 0;
    return true;
}

bool FlightRtNode::run(std::size_t max_cycles) {
    if (!init()) {
        return false;
    }

    constexpr std::uint64_t kTickUs = 1000;
    constexpr std::uint64_t kControlStepUs = 2500;
    constexpr std::uint64_t kSetpointTimeoutUs = 300000;
    constexpr std::uint64_t kHeartbeatTimeoutUs = 500000;
    constexpr std::uint64_t kReceiverTimeoutUs = 120000;
    constexpr std::uint32_t kLoopPeriodUs = static_cast<std::uint32_t>(kTickUs);

    std::uint32_t wake_tick = hal_rtos_tick_count();
    std::uint32_t loop_period_ticks = hal_rtos_us_to_ticks(kLoopPeriodUs);
    if (loop_period_ticks == 0u) {
        loop_period_ticks = 1u;
    }

    const std::uint64_t boot_us = monotonicNowUs();
    std::uint64_t last_control_us = boot_us;
    std::uint64_t next_control_due_us = boot_us + kControlStepUs;

    std::size_t cycle = 0;
    while ((max_cycles == 0u) || (cycle < max_cycles)) {
        ++cycle;
        const std::uint64_t now_us = monotonicNowUs();

        IncomingCommand command;
        if (!transport_->poll(command)) {
            enterFailsafe();
        }
        applyCommand(command, now_us);

        ReceiverSample receiver_sample{};
        if (!receiver_.poll(receiver_sample)) {
            enterFailsafe();
        } else {
            applyReceiver(receiver_sample, now_us);
        }

        const std::uint64_t safety_now_us = monotonicNowUs();
        if (controller_.armState() == proto::ArmState::Armed) {
            if ((safety_now_us - last_setpoint_us_) > kSetpointTimeoutUs) {
                enterFailsafe();
            }
            if (last_heartbeat_us_ != 0 && (safety_now_us - last_heartbeat_us_) > kHeartbeatTimeoutUs) {
                enterFailsafe();
            }
            if (!receiver_.hasLink(safety_now_us, kReceiverTimeoutUs)) {
                enterFailsafe();
            }
        }

        const std::uint64_t control_now_us = monotonicNowUs();
        if (control_now_us < next_control_due_us) {
            hal_rtos_delay_until(&wake_tick, loop_period_ticks);
            continue;
        }

        const std::uint64_t control_elapsed_us = control_now_us - last_control_us;
        last_control_us = control_now_us;
        do {
            next_control_due_us += kControlStepUs;
        } while (control_now_us >= next_control_due_us);

        const float dt_s = static_cast<float>(control_elapsed_us) / 1'000'000.0f;

        RawImuSample sample{};
        if (!sense_.sample(sample)) {
            enterFailsafe();
            hal_rtos_delay_until(&wake_tick, loop_period_ticks);
            continue;
        }

        AttitudeState state = estimator_.update(sample, dt_s);
        RawMagSample mag_sample{};
        if (sense_.sampleMag(mag_sample)) {
            state = estimator_.correctYawFromMag(mag_sample, dt_s);
        }
        const auto motor_pwm = controller_.update(state, dt_s);
        actuator_.output(motor_pwm);

        proto::RtReport report{};
        report.monotonic_us = monotonicNowUs();
        report.roll_deg = state.roll_deg;
        report.pitch_deg = state.pitch_deg;
        report.yaw_deg = state.yaw_deg;
        report.roll_rate_dps = state.roll_rate_dps;
        report.pitch_rate_dps = state.pitch_rate_dps;
        report.yaw_rate_dps = state.yaw_rate_dps;
        report.motor_pwm_us = motor_pwm;
        report.status_flags = proto::StatusImuHealthy;
        if (controller_.armState() == proto::ArmState::Armed) {
            report.status_flags |= proto::StatusArmed;
            report.status_flags |= proto::StatusRcHealthy;
        }
        if (controller_.armState() == proto::ArmState::Failsafe) {
            report.status_flags |= proto::StatusFailsafe;
        }
        report.seq = ++report_seq_;
        transport_->publish(report);

        hal_rtos_delay_until(&wake_tick, loop_period_ticks);
    }
    controller_.setArmState(proto::ArmState::Disarmed);
    actuator_.stop();
    return true;
}

void FlightRtNode::applyCommand(const IncomingCommand& command, std::uint64_t now_us) {
    if (command.emergency_stop) {
        enterFailsafe();
        return;
    }

    if (command.arm.has_value()) {
        const auto state = static_cast<proto::ArmState>(command.arm->state);
        controller_.setArmState(state);
        if (state != proto::ArmState::Armed) {
            actuator_.stop();
        }
    }

    if (command.setpoint.has_value()) {
        ControlSetpoint setpoint{};
        setpoint.throttle = command.setpoint->throttle;
        setpoint.roll_deg = command.setpoint->roll_deg;
        setpoint.pitch_deg = command.setpoint->pitch_deg;
        setpoint.yaw_rate_dps = command.setpoint->yaw_rate_dps;
        controller_.setSetpoint(setpoint);
        last_setpoint_us_ = now_us;
    }

    if (command.heartbeat.has_value()) {
        last_heartbeat_us_ = now_us;
    }

    if (command.config.has_value()) {
        controller_.applyConfig(*command.config);
        receiver_.applyConfig(*command.config);
    }
}

void FlightRtNode::applyReceiver(const ReceiverSample& sample, std::uint64_t now_us) {
    if (sample.failsafe_switch) {
        enterFailsafe();
        return;
    }
    if (!sample.link_ok) {
        return;
    }

    const bool arm_switch_high = sample.channels[4] > 1500;
    const bool throttle_low = sample.channels[2] <= 1100;
    if (arm_switch_high && throttle_low && controller_.armState() == proto::ArmState::Disarmed) {
        controller_.setArmState(proto::ArmState::Armed);
    }
    if (!arm_switch_high && controller_.armState() == proto::ArmState::Armed) {
        controller_.setArmState(proto::ArmState::Disarmed);
        actuator_.stop();
    }

    ControlSetpoint setpoint{};
    setpoint.roll_deg = mapChannelUs(sample.channels[0], -30.0f, 30.0f);
    setpoint.pitch_deg = mapChannelUs(sample.channels[1], -30.0f, 30.0f);
    setpoint.throttle = mapChannelUs(sample.channels[2], 0.0f, 1.0f);
    setpoint.yaw_rate_dps = mapChannelUs(sample.channels[3], -150.0f, 150.0f);
    controller_.setSetpoint(setpoint);
    last_setpoint_us_ = now_us;
}

void FlightRtNode::enterFailsafe() {
    controller_.setArmState(proto::ArmState::Failsafe);
    actuator_.stop();
}

std::uint64_t FlightRtNode::monotonicNowUs() const {
    return hal_micros();
}

}  // namespace amp::rt
