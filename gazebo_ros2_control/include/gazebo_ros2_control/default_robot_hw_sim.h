/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_

// ros_control
#if 0 //@todo
#include <control_toolbox/pid.hpp>
#endif
#if 0 //@todo
#include <hardware_interface/joint_command_interface.hpp>
#include <hardware_interface/robot_hw.hpp>
#endif

#include <joint_limits_interface/joint_limits.hpp>
#include <joint_limits_interface/joint_limits_interface.hpp>
#include <joint_limits_interface/joint_limits_rosparam.hpp>
#include <joint_limits_interface/joint_limits_urdf.hpp>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

// gazebo_ros_control
#include <gazebo_ros2_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>



namespace gazebo_ros2_control
{

class DefaultRobotHWSim : public gazebo_ros2_control::RobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    rclcpp::Node::SharedPtr& model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(rclcpp::Time time, rclcpp::Duration period);

  virtual void writeSim(rclcpp::Time time, rclcpp::Duration period);

  virtual void eStopActive(const bool active);

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};
protected:

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(size_t joint_nr,
                           const rclcpp::Node::SharedPtr& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit,
                           double *const vel_limit);

  unsigned int n_dof_;

#if 0 //@todo
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
#endif
#if 0 //@todo
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;
#endif
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<double> joint_vel_limits_;
  std::vector<ControlMethod> joint_control_methods_;
#if 0
  std::vector<control_toolbox::Pid> pid_controllers_;
#endif
  /// \brief stores the joint positions on ESTOP activation
  /// During ESTOP these positions will override the output position command value.
  std::vector<double> last_joint_position_command_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// \brief The current positions of the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_pos_stateh_;
  /// \brief The current velocities of the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_vel_stateh_;
  /// \brief The current effort forces applied to the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_eff_stateh_;

  /// \brief The current position command value (if control mode is POSITION) of the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_pos_cmdh_;
  /// \brief The current effort command value (if control mode is EFFORT) of the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_eff_cmdh_;
  /// \brief The current velocity command value (if control mode is VELOCITY) of the joints
  std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_vel_cmdh_;

  /// \brief The operational mode (active/inactive) state of the joints
  std::vector<hardware_interface::OperationMode> joint_opmodes_;

  /// \brief operational mode handles of the joints pointing to values in the joint_opmodes_ collection
  std::vector<hardware_interface::OperationModeHandle> joint_opmodehandles_;

  /// \brief Saturation limits for the joints if defined in the URDF or Node parameters
  std::vector<std::shared_ptr<joint_limits_interface::JointSaturationLimitHandle>> joint_limit_handles_;
  /// \brief Soft limits for the joints if defined in the URDF or Node parameters
  std::vector<std::shared_ptr<joint_limits_interface::JointSoftLimitsHandle>> joint_soft_limit_handles_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
};

typedef boost::shared_ptr<DefaultRobotHWSim> DefaultRobotHWSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_
