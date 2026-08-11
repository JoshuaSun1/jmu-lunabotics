#ifndef LB_HARDWARE__LUNABOT_DRIVE_HARDWARE_HPP_
#define LB_HARDWARE__LUNABOT_DRIVE_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "lb_hardware/drive_transport.hpp"
#include "rclcpp/macros.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rclcpp {
class Duration;
class Time;
} // namespace rclcpp

namespace rclcpp_lifecycle {
class State;
} // namespace rclcpp_lifecycle

namespace lb_hardware {

/// ros2_control system plugin for the four-wheel skid-steer drive boundary.
class LunabotDriveHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LunabotDriveHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  bool configure_transport();
  bool validate_joint_interfaces() const;
  bool disable_and_stop();
  bool command_values_are_finite() const;
  void zero_command_interfaces();

  std::unique_ptr<DriveTransport> transport_;
  std::vector<double> command_velocity_rad_s_;
  std::vector<double> position_rad_;
  std::vector<double> velocity_rad_s_;
  bool transport_connected_{false};
  bool command_output_enabled_{false};
  bool enable_on_activate_{false};
};

} // namespace lb_hardware

#endif // LB_HARDWARE__LUNABOT_DRIVE_HARDWARE_HPP_
