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

#include "control_toolbox/pid_ros.hpp"

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"

namespace gazebo_ros2_control
{

class GazeboSystem : public GazeboSystemInterface
{
public:
  // Documentation Inherited
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info)
  override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  hardware_interface::return_type start() override;

  // Documentation Inherited
  hardware_interface::return_type stop() override;

  // Documentation Inherited
  hardware_interface::return_type read() override;

  // Documentation Inherited
  hardware_interface::return_type write() override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model * const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions,
    sdf::ElementPtr sdf) override;

  /// \brief callback to get the e_stop signal from the ROS 2 topic
  void eStopCB(const std::shared_ptr<std_msgs::msg::Bool> e_stop_active);

private:
  /// \brief Degrees od freedom.
  unsigned int n_dof_;

  /// \brief e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_;

  /// \brief Emergency stop subscriber.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;

  /// \brief Gazebo Model Ptr.
  gazebo::physics::ModelPtr parent_model_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<ControlMethod> joint_control_methods_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current joint effort
  std::vector<double> joint_effort_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the last cmd joint position
  std::vector<double> last_joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief The current positions of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_pos_state_;

  /// \brief The current velocities of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_vel_state_;

  /// \brief The current effort forces applied to the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_eff_state_;

  /// \brief The position command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_pos_cmd_;

  /// \brief The velocity command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_vel_cmd_;

  /// \brief The effort command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_eff_cmd_;

  /// \brief vector with the PID of each joint.
  std::vector<control_toolbox::PidROS> pid_controllers_;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
