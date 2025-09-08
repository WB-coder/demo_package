#ifndef NAISCORP_ROBOT_SYSTEM_HPP_
#define NAISCORP_ROBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "naiscorp_hardware_interface/visibility_control.h"

#include "naiscorp_hardware_interface/naiscorp_comms.hpp"
#include "naiscorp_hardware_interface/wheel.hpp"

namespace naiscorp_robot
{
class NaiscorpRobotHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int wheel_diameter = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NaiscorpRobotHardware)

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  NAISCORP_ROBOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NAISCORP_ROBOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  NAISCORP_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  NAISCORP_ROBOT_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  naiscorp::MCUComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  
  // Timer-related members for 10ms interval execution
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_timer_execution_[2];

  // Define MAX_RPM as a constant instead of hardcoding 333
  unsigned char MAX_RPM = 37;
};

}  // namespace NAISCORP_ROBOT

#endif  // NAISCORP_ROBOT_SYSTEM_HPP_