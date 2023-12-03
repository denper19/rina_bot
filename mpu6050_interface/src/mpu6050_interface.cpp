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
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Initializing...");

  cfg.device = info_.hardware_parameters["device"];
  cfg.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

	RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Finished initialization");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MPU6050Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &orientation.qx));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &orientation.qy));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &orientation.qz));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &orientation.qw));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &angular_vel_x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &angular_vel_y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &angular_vel_z));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &linear_accel_x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_accel_y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_accel_z));



  return state_interfaces;
}

hardware_interface::CallbackReturn MPU6050Interface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  if (comms.connected())
  {
    comms.disconnect();
  }

  comms.connect(cfg.device, cfg.baud_rate, cfg.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MPU6050Interface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  if (comms.connected())
  {
    comms.disconnect();
  }

  RCLCPP_INFO(rclcpp::get_logger("MPU6050Interface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MPU6050Interface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  comms.read_imu_values(quaternion, accel_values, gyro_values);

  orientation.qx = quaternion[0];
  orientation.qy = quaternion[1];
  orientation.qz = quaternion[2];
  orientation.qw = quaternion[3];
	linear_accel_x = accel_values[1];
	linear_accel_y = accel_values[0];
	linear_accel_z = accel_values[2];
  angular_vel_x =  gyro_values[1];	
	angular_vel_y =  gyro_values[0];	
	angular_vel_z =  gyro_values[2];	

  return hardware_interface::return_type::OK;
}


} // namespace mpu6050_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mpu6050_interface::MPU6050Interface,
  hardware_interface::SensorInterface)