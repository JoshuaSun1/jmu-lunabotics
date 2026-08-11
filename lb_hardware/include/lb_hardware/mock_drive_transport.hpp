#ifndef LB_HARDWARE__MOCK_DRIVE_TRANSPORT_HPP_
#define LB_HARDWARE__MOCK_DRIVE_TRANSPORT_HPP_

#include "lb_hardware/drive_transport.hpp"

#include <chrono>
#include <cstddef>

namespace lb_hardware {

/// Configuration for deterministic Phase 2 mock transport and fault-injection tests.
struct MockDriveTransportConfig {
  std::chrono::nanoseconds command_timeout{std::chrono::milliseconds{250}};
  int communication_loss_after_reads{-1};
};

/// In-memory four-wheel transport; it never opens a device or commands physical hardware.
class MockDriveTransport final : public DriveTransport {
public:
  explicit MockDriveTransport(MockDriveTransportConfig config = {});

  bool connect() override;
  void disconnect() override;
  bool read_state(DriveState& state) override;
  bool write_command(const DriveCommand& command) override;
  bool set_enabled(bool enabled) override;
  bool stop() override;
  DriveFaults get_faults() const override;

  /// Advance mock time without sleeping; this is only for deterministic tests/simulation.
  void advance(std::chrono::nanoseconds period);

private:
  bool command_is_finite(const DriveCommand& command) const;
  void apply_elapsed(std::chrono::nanoseconds period);
  void zero_output();

  MockDriveTransportConfig config_;
  DriveCommand command_{};
  DriveState state_{};
  DriveFaults faults_{};
  std::chrono::nanoseconds command_age_{0};
  std::chrono::steady_clock::time_point last_read_time_{};
  std::size_t successful_reads_{0U};
  bool connected_{false};
  bool enabled_{false};
};

} // namespace lb_hardware

#endif // LB_HARDWARE__MOCK_DRIVE_TRANSPORT_HPP_
