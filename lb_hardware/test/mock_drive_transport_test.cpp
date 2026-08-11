#include "lb_hardware/mock_drive_transport.hpp"
#include "lb_hardware/real_drive_transport.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

bool values_are_zero(const lb_hardware::WheelValues& values) {
  for (const double value : values) {
    if (std::abs(value) > 1e-12) {
      return false;
    }
  }
  return true;
}

void test_disabled_start_and_mock_motion() {
  lb_hardware::MockDriveTransport mock;
  lb_hardware::DriveState state{};
  lb_hardware::DriveCommand command{};
  command.velocity_rad_s = {1.0, 1.0, 1.0, 1.0};

  expect(mock.connect(), "mock transport connects");
  expect(mock.write_command(command), "disabled mock accepts a command safely");
  mock.advance(std::chrono::milliseconds{20});
  expect(mock.read_state(state), "disabled mock returns state");
  expect(!state.output_enabled, "mock starts disabled");
  expect(values_are_zero(state.velocity_rad_s), "disabled mock has zero wheel velocity");

  expect(mock.set_enabled(true), "mock output can be explicitly enabled");
  expect(mock.write_command(command), "enabled mock accepts a command");
  mock.advance(std::chrono::milliseconds{20});
  expect(mock.read_state(state), "enabled mock returns state");
  expect(state.output_enabled, "enabled mock reports output enabled");
  for (std::size_t index = 0U; index < lb_hardware::kDriveWheelCount; ++index) {
    expect(state.velocity_rad_s[index] == 1.0, "mock wheel velocity is expressed in rad/s");
    expect(state.position_rad[index] > 0.0, "mock wheel position advances in rad");
  }

  expect(mock.stop(), "mock stop succeeds while connected");
  mock.advance(std::chrono::milliseconds{20});
  expect(mock.read_state(state), "mock state remains readable after stop");
  expect(values_are_zero(state.velocity_rad_s), "mock stop zeros all wheel output");
}

void test_timeout_and_communication_loss() {
  lb_hardware::MockDriveTransport timeout_mock(
      lb_hardware::MockDriveTransportConfig{std::chrono::milliseconds{1}, -1});
  lb_hardware::DriveState state{};
  lb_hardware::DriveCommand command{};
  command.velocity_rad_s = {1.0, 1.0, 1.0, 1.0};

  expect(timeout_mock.connect(), "timeout mock connects");
  expect(timeout_mock.set_enabled(true), "timeout mock enables explicitly");
  expect(timeout_mock.write_command(command), "timeout mock accepts initial command");
  timeout_mock.advance(std::chrono::milliseconds{2});
  expect(timeout_mock.read_state(state), "command timeout preserves communication state");
  expect(values_are_zero(state.velocity_rad_s), "command timeout zeros mock wheel output");
  expect(timeout_mock.get_faults().command_timeout, "command timeout is reported");

  lb_hardware::MockDriveTransport loss_mock(
      lb_hardware::MockDriveTransportConfig{std::chrono::milliseconds{250}, 0});
  expect(loss_mock.connect(), "fault-injection mock connects");
  expect(!loss_mock.read_state(state), "configured communication loss fails the first read");
  expect(loss_mock.get_faults().communication_lost, "communication loss is reported");
  expect(values_are_zero(state.velocity_rad_s), "communication loss fails with zero output");
}

void test_real_skeleton_fails_closed() {
  lb_hardware::RealDriveTransport real;
  lb_hardware::DriveState state{};
  lb_hardware::DriveCommand command{};
  command.velocity_rad_s = {1.0, 1.0, 1.0, 1.0};

  expect(!real.connect(), "real skeleton does not open an unspecified transport");
  expect(!real.set_enabled(true), "real skeleton cannot enable output");
  expect(!real.write_command(command), "real skeleton cannot emit a wheel command");
  expect(!real.read_state(state), "real skeleton cannot fabricate encoder state");
  expect(real.get_faults().protocol_unavailable, "real skeleton reports protocol unavailable");
  expect(real.get_faults().communication_lost, "real skeleton reports communication failure");
}

} // namespace

int main() {
  test_disabled_start_and_mock_motion();
  test_timeout_and_communication_loss();
  test_real_skeleton_fails_closed();
  return failures == 0 ? 0 : 1;
}
