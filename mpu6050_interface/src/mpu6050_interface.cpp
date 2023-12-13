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

  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &q.x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &q.y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &q.z));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &q.w));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &gx));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &gy));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &gz));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &ax));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &ay));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &az));
  
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
  int status = comms.read_imu_data(&tqx, &tqy, &tqz, &tqw, &tax, &tay, &taz, &tgx, &tgy, &tgz);

#ifdef _DEBUG_

  if(status < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"), "IMU timed out. Setting all IMU values to 0.");
  }

#endif
  
  q.x = tqx / 100.0f;
  q.y = tqy / 100.0f;
  q.z = tqz / 100.0f;
  q.w = tqw / 100.0f;
  ax = tax / 16384.0;
  ay = tay / 16384.0;
  az = taz / 16384.0;
  gx = tgx / 131.0;
  gy = tgy / 131.0;
  gz = tgz / 131.0;	

  return hardware_interface::return_type::OK;
}


} // namespace mpu6050_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mpu6050_interface::MPU6050Interface,
  hardware_interface::SensorInterface)