// Copyright (c) 2013, Open Source Robotics Foundation. All rights reserved.
// Copyright (c) 2013, The Johns Hopkins University. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gazebo_ros2_control/default_robot_hw_sim.hpp"
#include "urdf/model.h"

namespace gazebo_ros2_control
{
bool DefaultRobotHWSim::initSim(
  const std::string & robot_namespace,
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model * const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  nh_ = model_nh;

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_vel_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  joint_opmodes_.resize(n_dof_);
  joint_opmodehandles_.resize(n_dof_);

  joint_pos_stateh_.resize(n_dof_, hardware_interface::JointHandle("position"));
  joint_vel_stateh_.resize(n_dof_, hardware_interface::JointHandle("velocity"));
  joint_eff_stateh_.resize(n_dof_, hardware_interface::JointHandle("effort"));
  joint_pos_cmdh_.resize(n_dof_, hardware_interface::JointHandle("position_command"));
  joint_eff_cmdh_.resize(n_dof_, hardware_interface::JointHandle("effort_command"));
  joint_vel_cmdh_.resize(n_dof_, hardware_interface::JointHandle("velocity_command"));

  joint_limit_handles_.resize(n_dof_);

  // get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
  physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  // special handing since ODE uses a different set/get interface then the other engines
  usingODE = (physics_type_.compare("ode") == 0);

  // Initialize values
  for (unsigned int j = 0; j < n_dof_; j++) {
    //
    // Perform some validation on the URDF joint and actuator spec
    //

    // Check that this transmission has one joint
    if (transmissions[j].joints.empty()) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Transmission " << transmissions[j].name <<
          " has no associated joints.");
      continue;
    } else if (transmissions[j].joints.size() > 1) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Transmission " << transmissions[j].name <<
          " has more than one joint. Currently the default robot hardware simulation " <<
          " interface only supports one.");
      continue;
    }

    std::string joint_name = joint_names_[j] = transmissions[j].joints[0].name;

    std::vector<std::string> joint_interfaces = transmissions[j].joints[0].interfaces;
    if (joint_interfaces.empty() &&
      !(transmissions[j].actuators.empty()) &&
      !(transmissions[j].actuators[0].interfaces.empty()))
    {
      // TODO(anyone): Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators[0].interfaces;
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "The <hardware_interface> element of transmission " <<
          transmissions[j].name << " should be nested inside the <joint> element," <<
          " not <actuator>. The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty()) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Joint " << transmissions[j].name <<
          " of transmission " << transmissions[j].name <<
          " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
      continue;
    } else if (joint_interfaces.size() > 1) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Joint " << transmissions[j].name <<
          " of transmission " << transmissions[j].name <<
          " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one." <<
          "Using the first entry");
      // only a warning, allow joint to continue
    }

    std::string hardware_interface = joint_interfaces.front();
    // Decide what kind of command interface this actuator/joint has
    if (hardware_interface == "hardware_interface/EffortJointInterface") {
      joint_control_methods_[j] = EFFORT;
    } else if (hardware_interface == "hardware_interface/PositionJointInterface") {
      joint_control_methods_[j] = POSITION;
    } else if (hardware_interface == "hardware_interface/VelocityJointInterface") {
      joint_control_methods_[j] = VELOCITY;
    } else {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "No matching joint interface '" <<
          hardware_interface << "' for joint " << joint_name);
      RCLCPP_INFO(
        nh_->get_logger(),
        "    Expecting one of 'hardware_interface/{EffortJointInterface |"
        " PositionJointInterface | VelocityJointInterface}'");
      return false;
    }

    //
    // Accept this URDF joint as valid and link with the gazebo joint of the same name
    //

    // I think it's safe to only skip this joint and not abort if there is no match found
    gazebo::physics::JointPtr simjoint = parent_model->GetJoint(joint_name);
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }
    sim_joints_.push_back(simjoint);

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "Loading joint '" << joint_name << "' of type '" <<
        hardware_interface << "'");

    ///
    /// \brief a helper function for registering joint state or command handles
    ///
    auto register_joint_interface = [this, &joint_name, &j](
      const char * interface_name,
      std::vector<hardware_interface::JointHandle> & joint_iface_handles)
      {
        // set joint name and interface on our stored handle
        joint_iface_handles[j] = hardware_interface::JointHandle(joint_name, interface_name);

        // register the joint with the underlying RobotHardware implementation
        if (register_joint(
            joint_name, interface_name,
            0.0) != hardware_interface::return_type::OK)
        {
          RCLCPP_ERROR_STREAM(
            nh_->get_logger(), "cant register " << interface_name <<
              " state handle for joint " << joint_name);
          return false;
        }

        // now retrieve the handle with the bound value reference (bound directly to the
        // DynamicJointState msg in RobotHardware)
        if (get_joint_handle(joint_iface_handles[j]) != hardware_interface::return_type::OK) {
          RCLCPP_ERROR_STREAM(
            nh_->get_logger(), "state handle " << interface_name << " failure for joint " <<
              joint_name);
          return false;
        }

        // verify handle references a target value
        if (!joint_iface_handles[j]) {
          RCLCPP_ERROR_STREAM(
            nh_->get_logger(), interface_name << " handle for joint " << joint_name <<
              " is null");
          return false;
        }
        return true;
      };

    // register the state handles
    if (
      !register_joint_interface("position", joint_pos_stateh_) ||
      !register_joint_interface("velocity", joint_vel_stateh_) ||
      !register_joint_interface("effort", joint_eff_stateh_))
    {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "plugin aborting due to previous " << joint_name << " joint registration errors");
      return false;
    }

    // Register the command handle
    switch (joint_control_methods_[j]) {
      case EFFORT: register_joint_interface("effort_command", joint_eff_cmdh_); break;
      case POSITION: register_joint_interface("position_command", joint_pos_cmdh_); break;
      case VELOCITY: register_joint_interface("velocity_command", joint_vel_cmdh_); break;
    }

    // set joints operation mode to ACTIVE and register handle for controlling opmode
    joint_opmodes_[j] = hardware_interface::OperationMode::ACTIVE;
    joint_opmodehandles_[j] = hardware_interface::OperationModeHandle(
        joint_name, &joint_opmodes_[j]);
    if (register_operation_mode_handle(&joint_opmodehandles_[j]) !=
        hardware_interface::return_type::OK)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "cant register opmode handle for joint" << joint_name);
    }
  }

  // since handles references may have changed due to underlying DynamicJointState msg
  // vectors resizing and reallocating we need to get these handles again
  // any handles not registered are skipped, such as the command handles if they arent involved in the control method
  bindJointHandles(joint_pos_stateh_);
  bindJointHandles(joint_vel_stateh_);
  bindJointHandles(joint_eff_stateh_);
  bindJointHandles(joint_pos_cmdh_);
  bindJointHandles(joint_vel_cmdh_);
  bindJointHandles(joint_eff_cmdh_);


  //
  // Complete initialization of limits, PID controllers, etc now that registered handles are bound
  //
  for(size_t j=0; j < joint_names_.size(); j++) {
    auto simjoint = sim_joints_[j];
    assert(simjoint);

    // setup joint limits
    registerJointLimits(
      j,
      urdf_model,
      &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
      &joint_effort_limits_[j], &joint_vel_limits_[j]);
    if (joint_control_methods_[j] != EFFORT) {
      try {
        nh_->declare_parameter(transmissions[j].joints[0].name + ".p", 25.0);
        nh_->declare_parameter(transmissions[j].joints[0].name + ".i", 10.0);
        nh_->declare_parameter(transmissions[j].joints[0].name + ".d", 5.0);
        nh_->declare_parameter(transmissions[j].joints[0].name + ".i_clamp_max", 3.0);
        nh_->declare_parameter(transmissions[j].joints[0].name + ".i_clamp_min", 3.0);
        nh_->declare_parameter(transmissions[j].joints[0].name + ".antiwindup", false);
        pid_controllers_.push_back(
          control_toolbox::PidROS(nh_, transmissions[j].joints[0].name));
        if (pid_controllers_[j].initPid()) {
          switch (joint_control_methods_[j]) {
            case POSITION:
              joint_control_methods_[j] = POSITION_PID;
              RCLCPP_INFO(
                nh_->get_logger(), "joint %s is configured in POSITION_PID mode",
                transmissions[j].joints[0].name.c_str());
              break;
            case VELOCITY:
              joint_control_methods_[j] = VELOCITY_PID;
              RCLCPP_INFO(
                nh_->get_logger(), "joint %s is configured in VELOCITY_PID mode",
                transmissions[j].joints[0].name.c_str());
              break;
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo_ros2_control"), "%s", e.what());
      }
      simjoint->SetParam("fmax", 0, joint_effort_limits_[j]);
    }

    // set joints operation mode to ACTIVE and register handle for controlling opmode
    joint_opmodes_[j] = hardware_interface::OperationMode::ACTIVE;
    joint_opmodehandles_[j] = hardware_interface::OperationModeHandle(
      joint_names_[j], &joint_opmodes_[j]);
    if (register_operation_mode_handle(&joint_opmodehandles_[j]) !=
      hardware_interface::return_type::OK)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "cant register opmode handle for joint" << joint_names_[j]);
    }
  }

  // get the current state of the sim joint to initialize our ROS control joints
  // @note(guru-florida): perhaps we dont need this if readSim() is called after init anyway
  readSim(rclcpp::Time(), rclcpp::Duration(0, 0));

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void DefaultRobotHWSim::readSim(rclcpp::Time, rclcpp::Duration)
{
  for (unsigned int j = 0; j < joint_names_.size(); j++) {
    auto joint_handle = std::make_shared<hardware_interface::JointHandle>(
      joint_names_[j], "position");
    if (get_joint_handle(*joint_handle) == hardware_interface::return_type::OK) {
      double position = sim_joints_[j]->Position(0);
      if (joint_types_[j] == urdf::Joint::PRISMATIC) {
        joint_handle->set_value(position);
      } else {
        double prev_position = joint_handle->get_value();
        joint_handle->set_value(
          prev_position +
          angles::shortest_angular_distance(prev_position, position));
      }
    }
    joint_handle = std::make_shared<hardware_interface::JointHandle>(
      joint_names_[j], "velocity");
    if (get_joint_handle(*joint_handle) == hardware_interface::return_type::OK) {
      joint_handle->set_value(sim_joints_[j]->GetVelocity(0));
    }

    joint_handle = std::make_shared<hardware_interface::JointHandle>(
      joint_names_[j], "effort");
    if (get_joint_handle(*joint_handle) == hardware_interface::return_type::OK) {
      joint_handle->set_value(sim_joints_[j]->GetForce((unsigned int)(0)));
    }
  }
}

void DefaultRobotHWSim::writeSim(rclcpp::Time time, rclcpp::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_) {
    if (!last_e_stop_active_) {
      last_joint_position_command_.clear();
      std::transform(
        joint_pos_stateh_.begin(), joint_pos_stateh_.end(),
        std::back_inserter(last_joint_position_command_),
        [](const hardware_interface::JointHandle & ph) {return ph.get_value();});
      last_e_stop_active_ = true;
    }
    for (int i = 0; i < n_dof_; i++) {
      joint_pos_cmdh_[i].set_value(last_joint_position_command_[i]);
    }
  } else {
    last_e_stop_active_ = false;
  }
  for (auto & limit_handle : joint_limit_handles_) {
    if (limit_handle) {
      limit_handle->enforce_limits(period);
    }
  }
  for (unsigned int j = 0; j < n_dof_; j++) {
    switch (joint_control_methods_[j]) {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_eff_cmdh_[j].get_value();
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
        sim_joints_[j]->SetPosition(0, joint_pos_cmdh_[j].get_value(), true);
        break;

      case POSITION_PID:
        {
          double error;
          switch (joint_types_[j]) {
            case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(
                joint_pos_stateh_[j].get_value(),
                joint_pos_cmdh_[j].get_value(),
                joint_lower_limits_[j],
                joint_upper_limits_[j],
                error);
              break;
            case urdf::Joint::CONTINUOUS:
              error = angles::shortest_angular_distance(
                joint_pos_stateh_[j].get_value(),
                joint_pos_cmdh_[j].get_value());
              break;
            default:
              error = joint_pos_cmdh_[j].get_value() - joint_pos_stateh_[j].get_value();
          }

          const double effort_limit = joint_effort_limits_[j];

          const double effort = ignition::math::clamp(
            pid_controllers_[j].computeCommand(error, period),
            -effort_limit, effort_limit);

          sim_joints_[j]->SetForce(0, effort);
          sim_joints_[j]->SetParam("friction", 0, 0.0);
        }
        break;

      case VELOCITY:
        if (usingODE) {
          sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_vel_cmdh_[j].get_value());
        } else {
          sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_vel_cmdh_[j].get_value());
        }
        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_) {
          error = -joint_vel_stateh_[j].get_value();
        } else {
          error = joint_vel_cmdh_[j].get_value() - joint_vel_stateh_[j].get_value();
        }
        const double effort_limit = joint_effort_limits_[j];
        const double effort = ignition::math::clamp(
          pid_controllers_[j].computeCommand(error, period),
          -effort_limit, effort_limit);
        sim_joints_[j]->SetForce(0, effort);
        break;
    }
  }
}

void DefaultRobotHWSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void DefaultRobotHWSim::registerJointLimits(
  size_t joint_nr,
  const urdf::Model * const urdf_model,
  int * const joint_type, double * const lower_limit,
  double * const upper_limit, double * const effort_limit,
  double * const vel_limit)
{
  const std::string & joint_name = joint_names_[joint_nr];
  const ControlMethod ctrl_method = joint_control_methods_[joint_nr];

  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;

  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL) {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint->name, nh_, limits)) {
        has_limits = true;
      }
      if (joint_limits_interface::getSoftJointLimits(urdf_joint->name, nh_, soft_limits)) {
        has_soft_limits = true;
      }

      // @note (ddeng): these joint limits arent input into the node, so fetch them from the urdf
      // TODO(guru-florida): These are now a part of the node as of 2020-09-03,
      // so what one do we take?
      limits.min_position = urdf_joint->limits->lower;
      limits.max_position = urdf_joint->limits->upper;
      limits.has_position_limits = true;

      limits.max_velocity = urdf_joint->limits->velocity;
      limits.has_velocity_limits = limits.max_velocity == 0.0 ? false : true;

      limits.max_effort = urdf_joint->limits->effort;
      limits.has_effort_limits = limits.max_effort == 0.0 ? false : true;

      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
      *effort_limit = limits.max_effort;
      *vel_limit = limits.max_velocity;

      has_limits = true;
    }
  }

  if (joint_limits_interface::getJointLimits(joint_name, nh_, limits)) {
    has_limits = true;
  }

  if (!has_limits) {
    return;
  }

  if (*joint_type == urdf::Joint::UNKNOWN) {
    // Infer the joint type.

    if (limits.has_position_limits) {
      *joint_type = urdf::Joint::REVOLUTE;
    } else {
      if (limits.angle_wraparound) {
        *joint_type = urdf::Joint::CONTINUOUS;
      } else {
        *joint_type = urdf::Joint::PRISMATIC;
      }
    }
  }

  if (limits.has_position_limits) {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits) {
    *effort_limit = limits.max_effort;
  }

  if (has_soft_limits) {
    switch (ctrl_method) {
      case EFFORT:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::EffortJointSaturationHandle>(
            joint_pos_cmdh_[joint_nr], joint_vel_cmdh_[joint_nr], joint_eff_cmdh_[joint_nr],
            limits);
        }
        break;
      case POSITION:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::PositionJointSoftLimitsHandle>(
            joint_pos_stateh_[joint_nr], joint_pos_cmdh_[joint_nr], limits, soft_limits);
        }
        break;
      case VELOCITY:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::VelocityJointSaturationHandle>(
            joint_vel_stateh_[joint_nr], joint_vel_cmdh_[joint_nr], limits);
        }
        break;
    }
  } else {
    switch (ctrl_method) {
      case EFFORT:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::EffortJointSaturationHandle>(
            joint_pos_cmdh_[joint_nr], joint_vel_cmdh_[joint_nr], joint_eff_cmdh_[joint_nr],
            limits);
        }
        break;
      case POSITION:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::PositionJointSaturationHandle>(
            joint_pos_stateh_[joint_nr], joint_pos_cmdh_[joint_nr], limits);
        }
        break;
      case VELOCITY:
        {
          joint_limit_handles_[joint_nr] =
            std::make_unique<joint_limits_interface::VelocityJointSaturationHandle>(
            joint_vel_stateh_[joint_nr], joint_vel_cmdh_[joint_nr], limits);
        }
        break;
    }
  }
}

void DefaultRobotHWSim::bindJointHandles(std::vector<hardware_interface::JointHandle> & joint_iface_handles)
{
  for(auto& jh: joint_iface_handles)
  {
    // some handles, especially command handles, may not be registered so skip these
    if(jh.get_name().empty() || jh.get_interface_name().empty())
      continue;

    // now retrieve the handle with the bound value reference (bound directly to the
    // DynamicJointState msg in RobotHardware)
    if (get_joint_handle(jh) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR_STREAM(
          nh_->get_logger(), "state handle " << jh.get_interface_name() << " failure for joint " <<
                                             jh.get_name());
      continue;
    }

    // verify handle references a target value
    if (!jh) {
      RCLCPP_ERROR_STREAM(
          nh_->get_logger(), jh.get_interface_name() << " handle for joint " << jh.get_name() <<
                                                     " is null");
    }
  }
};
}  // namespace gazebo_ros2_control

PLUGINLIB_EXPORT_CLASS(gazebo_ros2_control::DefaultRobotHWSim, gazebo_ros2_control::RobotHWSim)
