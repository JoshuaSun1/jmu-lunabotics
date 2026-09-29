#include "lb_hardware/mock_drive_transport.hpp"

#include <algorithm>
#include <cmath>

namespace lb_hardware {

MockDriveTransport::MockDriveTransport(MockDriveTransportConfig config) : config_(config) {}

bool MockDriveTransport::connect() {
  if (config_.command_timeout.count() <= 0) {
    faults_.invalid_command = true;
    return false;
  }

  command_ = DriveCommand{};
  state_ = DriveState{};
  faults_ = DriveFaults{};
  command_age_ = config_.command_timeout;
  successful_reads_ = 0U;
  connected_ = true;
  enabled_ = false;
  last_read_time_ = std::chrono::steady_clock::now();
  state_.communication_ok = true;
  return true;
}

void MockDriveTransport::disconnect() {
  enabled_ = false;
  zero_output();
  connected_ = false;
  state_.communication_ok = false;
}

bool MockDriveTransport::read_state(DriveState& state) {
  if (!connected_) {
    faults_.communication_lost = true;
    state = state_;
    return false;
  }

  if (config_.communication_loss_after_reads >= 0 &&
      successful_reads_ >= static_cast<std::size_t>(config_.communication_loss_after_reads)) {
    enabled_ = false;
    zero_output();
    connected_ = false;
    state_.communication_ok = false;
    faults_.communication_lost = true;
    state = state_;
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  apply_elapsed(std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_read_time_));
  last_read_time_ = now;
  ++successful_reads_;
  state = state_;
  return true;
}

bool MockDriveTransport::write_command(const DriveCommand& command) {
  if (!connected_) {
    faults_.communication_lost = true;
    return false;
  }
  if (!command_is_finite(command)) {
    faults_.invalid_command = true;
    zero_output();
    return false;
  }

  command_age_ = std::chrono::nanoseconds{0};
  faults_.command_timeout = false;
  if (enabled_) {
    command_ = command;
  } else {
    command_ = DriveCommand{};
    zero_output();
  }
  return true;
}

bool MockDriveTransport::set_enabled(const bool enabled) {
  if (!connected_) {
    faults_.communication_lost = true;
    return false;
  }

  enabled_ = enabled;
  state_.output_enabled = enabled_;
  if (!enabled_) {
    command_ = DriveCommand{};
    zero_output();
  }
  return true;
}

bool MockDriveTransport::stop() {
  if (!connected_) {
    faults_.communication_lost = true;
    return false;
  }

  command_ = DriveCommand{};
  zero_output();
  return true;
}

DriveFaults MockDriveTransport::get_faults() const {
  return faults_;
}

void MockDriveTransport::advance(const std::chrono::nanoseconds period) {
  if (connected_ && period.count() > 0) {
    apply_elapsed(period);
  }
}

bool MockDriveTransport::command_is_finite(const DriveCommand& command) const {
  return std::all_of(command.velocity_rad_s.cbegin(), command.velocity_rad_s.cend(),
                     [](const double value) { return std::isfinite(value); });
}

void MockDriveTransport::apply_elapsed(const std::chrono::nanoseconds period) {
  command_age_ += period;
  const bool timed_out = command_age_ > config_.command_timeout;
  faults_.command_timeout = timed_out;

  if (!enabled_ || timed_out) {
    zero_output();
    return;
  }

  const double seconds = std::chrono::duration<double>(period).count();
  for (std::size_t index = 0U; index < kDriveWheelCount; ++index) {
    state_.velocity_rad_s[index] = command_.velocity_rad_s[index];
    state_.position_rad[index] += command_.velocity_rad_s[index] * seconds;
  }
  state_.communication_ok = true;
  state_.output_enabled = true;
}

void MockDriveTransport::zero_output() {
  state_.velocity_rad_s.fill(0.0);
  state_.output_enabled = enabled_;
}

} // namespace lb_hardware
