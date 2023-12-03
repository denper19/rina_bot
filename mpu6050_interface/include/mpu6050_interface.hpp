// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#ifndef MPU6050_ARDUINO__IMU_SYSTEM_HPP_
#define MPU6050_ARDUINO__IMU_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "visibility_control_imu.h"
#include "arduino_comms.hpp"

typedef struct Quaternion
{
  double qx;
  double qy;
  double qz;
  double qw;
} Quaternion;

struct Config
{
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
};

namespace mpu6050_interface
{
class MPU6050Interface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MPU6050Interface);

  MPU6050_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MPU6050_ARDUINO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MPU6050_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MPU6050_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MPU6050_ARDUINO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MPU6050_ARDUINO_PUBLIC
  void euler_to_quat();

private:

  ArduinoComms comms;
  Config cfg;

  Quaternion orientation;

  double linear_accel_x;
  double linear_accel_y;
  double linear_accel_z;

  double angular_vel_x;
  double angular_vel_y;
  double angular_vel_z;

  float quaternion[4];
  float gyro_values[3];
  float accel_values[3];


  // Store the sensor states for the simulated robot
  std::vector<double> hw_sensor_states_;
};

}  // mpu6050_interface

#endif  // MPU6050_ARDUINO__IMU_SYSTEM_HPP_