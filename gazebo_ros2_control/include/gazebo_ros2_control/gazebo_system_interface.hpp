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


namespace gazebo_ros2_control
{

template<class ENUM, class UNDERLYING = typename std::underlying_type<ENUM>::type>
class SafeEnum
{
public:
  SafeEnum()
  : mFlags(0) {}
  explicit SafeEnum(ENUM singleFlag)
  : mFlags(singleFlag) {}
  SafeEnum(const SafeEnum & original)
  : mFlags(original.mFlags) {}

  SafeEnum & operator|=(ENUM addValue) {mFlags |= addValue; return *this;}
  SafeEnum operator|(ENUM addValue) {SafeEnum result(*this); result |= addValue; return result;}
  SafeEnum & operator&=(ENUM maskValue) {mFlags &= maskValue; return *this;}
  SafeEnum operator&(ENUM maskValue) {SafeEnum result(*this); result &= maskValue; return result;}
  SafeEnum operator~() {SafeEnum result(*this); result.mFlags = ~result.mFlags; return result;}
  explicit operator bool() {return mFlags != 0;}

protected:
  UNDERLYING mFlags;
};

// SystemInterface provides API-level access to read and command joint properties.
class GazeboSystemInterface
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  /// \brief Initialize the system interface
  /// param[in] model_nh pointer to the ros2 node
  /// param[in] parent_model pointer to the model
  /// param[in] control_hardware vector filled with information about robot's control resources
  /// param[in] sdf pointer to the SDF
  virtual bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) = 0;

  // Methods used to control a joint.
  enum ControlMethod_
  {
    NONE      = 0,
    POSITION  = (1 << 0),
    VELOCITY  = (1 << 1),
    EFFORT    = (1 << 2),
  };

  typedef SafeEnum<enum ControlMethod_> ControlMethod;

protected:
  rclcpp::Node::SharedPtr nh_;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_
