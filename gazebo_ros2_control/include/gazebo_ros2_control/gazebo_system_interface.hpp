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


#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/physics.hh"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include "transmission_interface/transmission_info.hpp"

// URDF
#include "urdf/model.h"

namespace gazebo_ros2_control
{

class GazeboSystemInterface
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  /// \brief Initilize the system interface
  /// param[in] model_nh pointer to the ros2 node
  /// param[in] parent_model pointer to the model
  /// param[in] urdf_model pointer to the URDF
  /// param[in] transmissions availables in the model
  /// param[in] sdf pointer to the SDF
  virtual bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model * const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions,
    sdf::ElementPtr sdf) = 0;

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

protected:
  rclcpp::Node::SharedPtr nh_;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_
