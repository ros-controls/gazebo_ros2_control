// Copyright 2021 Open Source Robotics Foundation, Inc.
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


#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"

namespace gazebo_ros2_control
{
// Forward declaration
class GazeboSystemPrivate;

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class GazeboSystem : public GazeboSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info)
  override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type read() override;

  // Documentation Inherited
  hardware_interface::return_type write() override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  /// \brief Private data class
  std::unique_ptr<GazeboSystemPrivate> dataPtr;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
