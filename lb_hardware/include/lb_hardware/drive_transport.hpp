#ifndef LB_HARDWARE__DRIVE_TRANSPORT_HPP_
#define LB_HARDWARE__DRIVE_TRANSPORT_HPP_

#include <array>
#include <cstddef>

namespace lb_hardware {

/// The Phase 2 hardware boundary has one state and one command per wheel joint.
inline constexpr std::size_t kDriveWheelCount = 4U;
using WheelValues = std::array<double, kDriveWheelCount>;

/// Wheel velocity commands in rad/s at the ROS/transport boundary.
struct DriveCommand {
  WheelValues velocity_rad_s{};
};

/// Encoder-derived wheel state in rad and rad/s at the ROS/transport boundary.
struct DriveState {
  WheelValues position_rad{};
  WheelValues velocity_rad_s{};
  bool communication_ok{false};
  bool output_enabled{false};
};

/// Transport-level faults. These do not replace the future safety supervisor.
struct DriveFaults {
  bool communication_lost{false};
  bool command_timeout{false};
  bool invalid_command{false};
  bool protocol_unavailable{false};
};

/// Abstract MCU transport; it deliberately contains no CAN, serial, or PWM assumptions.
class DriveTransport {
public:
  virtual ~DriveTransport() = default;

  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool read_state(DriveState& state) = 0;
  virtual bool write_command(const DriveCommand& command) = 0;
  virtual bool set_enabled(bool enabled) = 0;
  virtual bool stop() = 0;
  virtual DriveFaults get_faults() const = 0;
};

} // namespace lb_hardware

#endif // LB_HARDWARE__DRIVE_TRANSPORT_HPP_
