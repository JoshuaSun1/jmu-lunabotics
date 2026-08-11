#ifndef LB_HARDWARE__REAL_DRIVE_TRANSPORT_HPP_
#define LB_HARDWARE__REAL_DRIVE_TRANSPORT_HPP_

#include "lb_hardware/drive_transport.hpp"

namespace lb_hardware {

/// Fail-closed placeholder until DRIVE-03 and SAFE-01 define a reviewed MCU protocol.
class RealDriveTransport final : public DriveTransport {
public:
  bool connect() override;
  void disconnect() override;
  bool read_state(DriveState& state) override;
  bool write_command(const DriveCommand& command) override;
  bool set_enabled(bool enabled) override;
  bool stop() override;
  DriveFaults get_faults() const override;

private:
  DriveFaults faults_{false, false, false, true};
};

} // namespace lb_hardware

#endif // LB_HARDWARE__REAL_DRIVE_TRANSPORT_HPP_
