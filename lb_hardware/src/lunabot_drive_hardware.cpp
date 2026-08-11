#include "lb_hardware/lunabot_drive_hardware.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lb_hardware/mock_drive_transport.hpp"
#include "lb_hardware/real_drive_transport.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lb_hardware {
namespace {

constexpr char kTransportTypeParameter[] = "transport_type";
constexpr char kEnableOnActivateParameter[] = "enable_on_activate";
constexpr char kMockFailureAfterReadsParameter[] = "mock_communication_loss_after_reads";
constexpr char kMockCommandTimeoutParameter[] = "mock_command_timeout_s";

bool parse_bool(const std::string& value, bool& parsed) {
  if (value == "true") {
    parsed = true;
    return true;
  }
  if (value == "false") {
    parsed = false;
    return true;
  }
  return false;
}

bool has_named_interface(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                         const std::string& interface_name) {
  return std::any_of(interfaces.cbegin(), interfaces.cend(),
                     [&interface_name](const hardware_interface::InterfaceInfo& interface) {
                       return interface.name == interface_name;
                     });
}

} // namespace

hardware_interface::CallbackReturn
LunabotDriveHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!validate_joint_interfaces()) {
    RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                 "LunabotDriveHardware requires exactly four velocity-commanded wheel joints with "
                 "position and velocity state interfaces.");
    return CallbackReturn::ERROR;
  }

  const auto enable_parameter = info_.hardware_parameters.find(kEnableOnActivateParameter);
  if (enable_parameter == info_.hardware_parameters.cend() ||
      !parse_bool(enable_parameter->second, enable_on_activate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                 "The '%s' hardware parameter must be explicitly 'true' or 'false'.",
                 kEnableOnActivateParameter);
    return CallbackReturn::ERROR;
  }
  if (!configure_transport()) {
    return CallbackReturn::ERROR;
  }

  command_velocity_rad_s_.assign(kDriveWheelCount, 0.0);
  position_rad_.assign(kDriveWheelCount, 0.0);
  velocity_rad_s_.assign(kDriveWheelCount, 0.0);
  transport_connected_ = false;
  command_output_enabled_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LunabotDriveHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kDriveWheelCount * 2U);
  for (std::size_t index = 0U; index < kDriveWheelCount; ++index) {
    const auto& joint = info_.joints[index];
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                                  &position_rad_[index]);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                  &velocity_rad_s_[index]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LunabotDriveHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kDriveWheelCount);
  for (std::size_t index = 0U; index < kDriveWheelCount; ++index) {
    command_interfaces.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
                                    &command_velocity_rad_s_[index]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
LunabotDriveHardware::on_activate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  zero_command_interfaces();
  if (!transport_ || !transport_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                 "Drive transport connection failed; the hardware component remains fail-closed.");
    return CallbackReturn::ERROR;
  }

  transport_connected_ = true;
  if (!transport_->stop() || !transport_->set_enabled(enable_on_activate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                 "Drive transport could not enter its explicit disabled/enabled state safely.");
    transport_->disconnect();
    transport_connected_ = false;
    return CallbackReturn::ERROR;
  }

  command_output_enabled_ = enable_on_activate_;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LunabotDriveHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  const bool stopped = disable_and_stop();
  if (transport_) {
    transport_->disconnect();
  }
  transport_connected_ = false;
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::return_type LunabotDriveHardware::read(const rclcpp::Time& time,
                                                           const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  DriveState state{};
  if (!transport_connected_ || !transport_ || !transport_->read_state(state) ||
      !state.communication_ok) {
    disable_and_stop();
    return hardware_interface::return_type::ERROR;
  }

  position_rad_.assign(state.position_rad.cbegin(), state.position_rad.cend());
  velocity_rad_s_.assign(state.velocity_rad_s.cbegin(), state.velocity_rad_s.cend());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LunabotDriveHardware::write(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  if (!transport_connected_ || !transport_) {
    return hardware_interface::return_type::ERROR;
  }

  DriveCommand command{};
  if (command_output_enabled_) {
    if (!command_values_are_finite()) {
      disable_and_stop();
      return hardware_interface::return_type::ERROR;
    }
    std::copy(command_velocity_rad_s_.cbegin(), command_velocity_rad_s_.cend(),
              command.velocity_rad_s.begin());
  }

  if (!transport_->write_command(command)) {
    disable_and_stop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool LunabotDriveHardware::configure_transport() {
  const auto transport_parameter = info_.hardware_parameters.find(kTransportTypeParameter);
  if (transport_parameter == info_.hardware_parameters.cend()) {
    RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"), "Missing required '%s' hardware parameter.",
                 kTransportTypeParameter);
    return false;
  }

  if (transport_parameter->second == "mock") {
    const auto timeout_parameter = info_.hardware_parameters.find(kMockCommandTimeoutParameter);
    const auto failure_parameter = info_.hardware_parameters.find(kMockFailureAfterReadsParameter);
    if (timeout_parameter == info_.hardware_parameters.cend() ||
        failure_parameter == info_.hardware_parameters.cend()) {
      RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                   "Mock transport requires '%s' and '%s' parameters.",
                   kMockCommandTimeoutParameter, kMockFailureAfterReadsParameter);
      return false;
    }

    try {
      const double timeout_seconds = std::stod(timeout_parameter->second);
      const int failure_after_reads = std::stoi(failure_parameter->second);
      if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0 || failure_after_reads < -1) {
        throw std::invalid_argument("out-of-range mock transport parameter");
      }
      const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(timeout_seconds));
      transport_ = std::make_unique<MockDriveTransport>(
          MockDriveTransportConfig{timeout, failure_after_reads});
      return true;
    } catch (const std::exception&) {
      RCLCPP_ERROR(rclcpp::get_logger("lb_hardware"),
                   "Mock transport timeout must be finite and > 0 s; communication loss reads must "
                   "be -1 or >= 0.");
      return false;
    }
  }

  if (transport_parameter->second == "real") {
    transport_ = std::make_unique<RealDriveTransport>();
    return true;
  }

  RCLCPP_ERROR(
      rclcpp::get_logger("lb_hardware"),
      "Unsupported transport_type '%s'; expected 'mock' or the fail-closed 'real' skeleton.",
      transport_parameter->second.c_str());
  return false;
}

bool LunabotDriveHardware::validate_joint_interfaces() const {
  if (info_.joints.size() != kDriveWheelCount) {
    return false;
  }

  return std::all_of(info_.joints.cbegin(), info_.joints.cend(), [](const auto& joint) {
    return joint.command_interfaces.size() == 1U && joint.state_interfaces.size() == 2U &&
           has_named_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) &&
           has_named_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) &&
           has_named_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  });
}

bool LunabotDriveHardware::disable_and_stop() {
  zero_command_interfaces();
  command_output_enabled_ = false;
  if (!transport_) {
    transport_connected_ = false;
    return false;
  }
  const bool stopped = transport_->stop();
  const bool disabled = transport_->set_enabled(false);
  transport_connected_ = false;
  return stopped && disabled;
}

bool LunabotDriveHardware::command_values_are_finite() const {
  return std::all_of(command_velocity_rad_s_.cbegin(), command_velocity_rad_s_.cend(),
                     [](const double value) { return std::isfinite(value); });
}

void LunabotDriveHardware::zero_command_interfaces() {
  std::fill(command_velocity_rad_s_.begin(), command_velocity_rad_s_.end(), 0.0);
}

} // namespace lb_hardware

PLUGINLIB_EXPORT_CLASS(lb_hardware::LunabotDriveHardware, hardware_interface::SystemInterface)
