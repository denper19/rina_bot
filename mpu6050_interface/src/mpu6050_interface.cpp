#include "mpu6050_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mpu6050_interface
{

hardware_interface::CallbackReturn MPU6050Interface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MPU6050Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn MPU6050Interface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("MPU6050Interface"), "Activating ...please wait...");


  RCLCPP_INFO(
    rclcpp::get_logger("MPU6050Interface"), "Successfully activated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MPU6050Interface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("MPU6050Interface"), "Deactivating ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("MPU6050Interface"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MPU6050Interface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Reading...");

  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    // Simulate RRBot's sensor data
    hw_sensor_states_[i] = 35.5f;
    // RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Got state %e for sensor %u!", hw_sensor_states_[i], i);
  }
  // RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}


} // namespace mpu6050_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mpu6050_interface::MPU6050Interface,
  hardware_interface::SensorInterface)