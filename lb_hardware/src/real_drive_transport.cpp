#include "lb_hardware/real_drive_transport.hpp"

namespace lb_hardware {

bool RealDriveTransport::connect() {
  // TODO(DRIVE-03): implement only after the MCU topology, framing, heartbeat,
  // fault semantics, and encoder scaling have been reviewed and documented.
  faults_.communication_lost = true;
  return false;
}

void RealDriveTransport::disconnect() {
  faults_.communication_lost = true;
}

bool RealDriveTransport::read_state(DriveState& state) {
  // TODO(DRIVE-03): decode reviewed MCU state into radians and radians/second.
  state = DriveState{};
  faults_.communication_lost = true;
  return false;
}

bool RealDriveTransport::write_command(const DriveCommand& command) {
  (void)command;
  // TODO(DRIVE-03): encode a reviewed high-level wheel command; never motor phases.
  faults_.communication_lost = true;
  return false;
}

bool RealDriveTransport::set_enabled(const bool enabled) {
  (void)enabled;
  // TODO(SAFE-01): implement an explicit, reviewed enable/disable acknowledgement path.
  faults_.communication_lost = true;
  return false;
}

bool RealDriveTransport::stop() {
  // TODO(DRIVE-03): send a reviewed zero-command/stop packet and require acknowledgement.
  faults_.communication_lost = true;
  return false;
}

DriveFaults RealDriveTransport::get_faults() const {
  return faults_;
}

} // namespace lb_hardware
