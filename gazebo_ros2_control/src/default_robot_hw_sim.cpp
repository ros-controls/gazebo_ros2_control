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
#include <string>
#include <vector>

#include "gazebo_ros2_control/default_robot_hw_sim.hpp"
#include "urdf/model.h"

namespace gazebo_ros2_control
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("default_robot_hw_sim");

bool DefaultRobotHWSim::initSim(
  const std::string & robot_namespace,
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model * const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  rclcpp::Node::SharedPtr & joint_limit_nh = model_nh;

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_vel_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
#if 0
  pid_controllers_.resize(n_dof_);
#endif
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  joint_opmodes_.resize(n_dof_);

  joint_states_.resize(n_dof_);
  joint_cmds_.resize(n_dof_);
  joint_opmodehandles_.resize(n_dof_);
  joint_eff_cmdhandle_.resize(n_dof_);
  joint_vel_cmdhandle_.resize(n_dof_);

  joint_pos_limit_handles_.resize(n_dof_);
  joint_pos_soft_limit_handles_.resize(n_dof_);
  joint_eff_limit_handles_.resize(n_dof_);
  joint_eff_soft_limit_handles_.resize(n_dof_);
  joint_vel_limit_handles_.resize(n_dof_);

  // Initialize values
  for (unsigned int j = 0; j < n_dof_; j++) {
    // Check that this transmission has one joint
    if (transmissions[j].joints_.empty()) {
      std::cerr << "Transmission " << transmissions[j].name_ <<
        " has no associated joints.";
      continue;
    } else if (transmissions[j].joints_.size() > 1) {
      std::cerr << "Transmission " << transmissions[j].name_ <<
        " has more than one joint. Currently the default robot hardware simulation " <<
        " interface only supports one.";
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
      !(transmissions[j].actuators_.empty()) &&
      !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO(anyone): Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      std::cerr << "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ <<
        " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.";
    }
    if (joint_interfaces.empty()) {
      std::cerr << "Joint " << transmissions[j].name_ <<
        " of transmission " << transmissions[j].name_ <<
        " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.";
      continue;
    } else if (joint_interfaces.size() > 1) {
      std::cerr << "Joint " << transmissions[j].name_ <<
        " of transmission " << transmissions[j].name_ <<
        " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. "
        "Using the first entry";
      // continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string & hardware_interface = joint_interfaces.front();

    // Debug
    std::cerr << "Loading joint '" << joint_names_[j] <<
      "' of type '" << hardware_interface << "'" << std::endl;

    // Create joint state interface for all joints
#if 0  // @todo
    js_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));
#endif

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointStateHandle joint_handle;
    if (hardware_interface == "EffortJointInterface" ||
      hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
#if 0  // @todo
      joint_handle = hardware_interface::JointHandle(
        js_interface_.getHandle(joint_names_[j]),
        &joint_effort_command_[j]);

      ej_interface_.registerHandle(joint_handle);
#endif
    } else {
      if (hardware_interface == "PositionJointInterface" ||
        hardware_interface == "hardware_interface/PositionJointInterface")
      {
        // Create position joint interface
        joint_control_methods_[j] = POSITION;
#if 0  // @todo
        joint_handle = hardware_interface::JointHandle(
          js_interface_.getHandle(joint_names_[j]),
          &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);
#endif
      } else {
        if (hardware_interface == "VelocityJointInterface" ||
          hardware_interface == "hardware_interface/VelocityJointInterface")
        {
          // Create velocity joint interface
          joint_control_methods_[j] = VELOCITY;

#if 0  // @todo
          joint_handle = hardware_interface::JointHandle(
            js_interface_.getHandle(joint_names_[j]),
            &joint_velocity_command_[j]);
          vj_interface_.registerHandle(joint_handle);
#endif
        } else {
          std::cerr << "No matching hardware interface found for '" <<
            hardware_interface << "' while loading interfaces for " << joint_names_[j] << std::endl;
          return false;
        }
      }
    }
    if (hardware_interface == "EffortJointInterface" ||
      hardware_interface == "PositionJointInterface" ||
      hardware_interface == "VelocityJointInterface")
    {
      std::cerr << "Deprecated syntax, please prepend 'hardware_interface/' to '" <<
        hardware_interface << "' within the <hardwareInterface> tag in joint '" <<
        joint_names_[j] << "'." << std::endl;
    }

    // Get the gazebo joint that corresponds to the robot joint.
    // RCLCPP_DEBUG_STREAM(LOGGER, "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint) {
      std::cerr << "This robot has a joint named \"" << joint_names_[j] <<
        "\" which is not in the gazebo model." << std::endl;
      return false;
    }
    sim_joints_.push_back(joint);
    joint_position_[j] = joint->Position(0);
    joint_velocity_[j] = joint->GetVelocity(0);

    // get physics engine type
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

    physics_type_ = physics->GetType();
    if (physics_type_.empty()) {
      std::cerr << "No physics type found." << std::endl;
    }

    registerJointLimits(
      joint_names_[j], joint_handle, joint_control_methods_[j],
      joint_limit_nh, urdf_model,
      &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
      &joint_effort_limits_[j], &joint_vel_limits_[j]);
    if (joint_control_methods_[j] != EFFORT) {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetParam("vel") to control the joint.
#if 0
      const ros::NodeHandle nh(robot_namespace + "//pid_gains/" +
        joint_names_[j]);
#endif
#if 0
      if (pid_controllers_[j].init(nh)) {
        switch (joint_control_methods_[j]) {
          case POSITION:
            joint_control_methods_[j] = POSITION_PID;
            break;
          case VELOCITY:
            joint_control_methods_[j] = VELOCITY_PID;
            break;
        }
      } else {}
#endif
      {
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
      }
    }
    joint_states_[j] = hardware_interface::JointStateHandle(
      joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]);
    if (register_joint_state_handle(&joint_states_[j]) != hardware_interface::HW_RET_OK) {
      RCLCPP_WARN_ONCE(LOGGER, "cant register jointstatehandle");
    }

    joint_cmds_[j] = hardware_interface::JointCommandHandle(
      joint_names_[j], &joint_position_command_[j]);
    if (register_joint_command_handle(&joint_cmds_[j]) != hardware_interface::HW_RET_OK) {
      RCLCPP_WARN_ONCE(LOGGER, "cant register jointcommandhandle");
    }

    joint_opmodehandles_[j] = hardware_interface::OperationModeHandle(
      joint_names_[j], &joint_opmodes_[j]);
    if (register_operation_mode_handle(&joint_opmodehandles_[j]) != hardware_interface::HW_RET_OK) {
      RCLCPP_WARN_ONCE(LOGGER, "cant register jointopmodehandle");
    }

    joint_limits_interface::JointLimits limits;  // hack, refactor registerjointhandle
    limits.has_position_limits = true;
    limits.min_position = joint_lower_limits_[j];
    limits.max_position = joint_upper_limits_[j];
    limits.max_velocity = joint_vel_limits_[j];
    limits.has_velocity_limits = true;
    limits.max_effort = joint_effort_limits_[j];
    limits.has_effort_limits = true;

    joint_pos_limit_handles_[j] = joint_limits_interface::PositionJointSaturationHandle(
      joint_states_[j], joint_cmds_[j], limits);

    joint_eff_cmdhandle_[j] = hardware_interface::JointCommandHandle(
      joint_names_[j], &joint_effort_command_[j]);
    // should be register_joint_effort_command_handle, but there's only 1 buffer for now
    if (register_joint_command_handle(&joint_eff_cmdhandle_[j]) != hardware_interface::HW_RET_OK) {
      RCLCPP_WARN_ONCE(LOGGER, "cant register jointcommandhandle");
    }

    joint_eff_limit_handles_[j] = joint_limits_interface::EffortJointSaturationHandle(
      joint_states_[j], joint_eff_cmdhandle_[j], limits);

    joint_vel_cmdhandle_[j] = hardware_interface::JointCommandHandle(
      joint_names_[j], &joint_velocity_command_[j]);
    if (register_joint_command_handle(&joint_vel_cmdhandle_[j]) != hardware_interface::HW_RET_OK) {
      std::cerr << "cant register jointcommandhandle" << std::endl;
    }

    joint_vel_limit_handles_[j] = joint_limits_interface::VelocityJointSaturationHandle(
      joint_states_[j], joint_vel_cmdhandle_[j], limits);
    // register not implemented
  }
#if 0  // @todo
  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
#endif
  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void DefaultRobotHWSim::readSim(rclcpp::Time time, rclcpp::Duration period)
{
  for (unsigned int j = 0; j < n_dof_; j++) {
    // Gazebo has an interesting API...
    double position = sim_joints_[j]->Position(0);
    if (joint_types_[j] == urdf::Joint::PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(
        joint_position_[j],
        position);
    }
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }
}

void DefaultRobotHWSim::writeSim(rclcpp::Time time, rclcpp::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_) {
    if (!last_e_stop_active_) {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  } else {
    last_e_stop_active_ = false;
  }
#if 0  // @todo
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
#else
  for (auto eff_limit_handle : joint_eff_limit_handles_) {
    eff_limit_handle.enforceLimits(period);
  }
  for (auto pos_limit_handle : joint_pos_limit_handles_) {
    pos_limit_handle.enforceLimits(period);
  }
  for (auto vel_limit_handle : joint_vel_limit_handles_) {
    vel_limit_handle.enforceLimits(period);
  }

#endif
  for (unsigned int j = 0; j < n_dof_; j++) {
    switch (joint_control_methods_[j]) {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
        sim_joints_[j]->SetPosition(0, joint_position_command_[j], true);
        break;

      case POSITION_PID:
        {
          double error;
          switch (joint_types_[j]) {
            case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(
                joint_position_[j],
                joint_position_command_[j],
                joint_lower_limits_[j],
                joint_upper_limits_[j],
                error);
              break;
            case urdf::Joint::CONTINUOUS:
              error = angles::shortest_angular_distance(
                joint_position_[j],
                joint_position_command_[j]);
              break;
            default:
              error = joint_position_command_[j] - joint_position_[j];
          }

          const double effort_limit = joint_effort_limits_[j];
          // TODO(anyone): Restored this when PID controllers is available
          // const double effort = std::clamp(pid_controllers_[j].computeCommand(error, period),
          //                             -effort_limit, effort_limit);
          const double effort = 0.0;
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case VELOCITY:
        if (physics_type_.compare("ode") == 0) {
          sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
        } else {
          sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
        }
        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_) {
          error = -joint_velocity_[j];
        } else {
          error = joint_velocity_command_[j] - joint_velocity_[j];
        }
        const double effort_limit = joint_effort_limits_[j];
        // TODO(anyone): Restored this when PID controllers is available
        // const double effort = std::clamp(pid_controllers_[j].computeCommand(error, period),
        //                             -effort_limit, effort_limit);
        const double effort = 0.0;
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
  const std::string & joint_name,
  const hardware_interface::JointStateHandle & joint_handle,
  const ControlMethod ctrl_method,
  const rclcpp::Node::SharedPtr & joint_limit_nh,
  const urdf::Model * const urdf_model,
  int * const joint_type, double * const lower_limit,
  double * const upper_limit, double * const effort_limit,
  double * const vel_limit)
{
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
      if (joint_limits_interface::getJointLimits(urdf_joint->name, joint_limit_nh, limits)) {
        has_limits = true;
      }
      if (joint_limits_interface::getSoftJointLimits(
          urdf_joint->name, joint_limit_nh,
          soft_limits))
      {
        has_soft_limits = true;
      }

      // @note (ddeng): these joint limits arent input into the node, so fetch them from the urdf
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

      // urdf_joint->safety->k_position;
      has_limits = true;
    }
  }

  // Get limits from the parameter server.
  // @note: no longer using parameter servers
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits)) {
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
#if 0  // @todo
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
#endif
        }
        break;
      case POSITION:
        {
#if 0  // @todo
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
#endif
        }
        break;
      case VELOCITY:
        {
#if 0  // @todo
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
#endif
        }
        break;
    }
  } else {
    switch (ctrl_method) {
      case EFFORT:
        {
#if 0
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);

          ej_sat_interface_.registerHandle(sat_handle);
#endif
        }
        break;
      case POSITION:
        {
#if 0  // @todo
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);

          pj_sat_interface_.registerHandle(sat_handle);
#endif
        }
        break;
      case VELOCITY:
        {
#if 0  // @todo
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
#endif
        }
        break;
    }
  }
}

}  // namespace gazebo_ros2_control

PLUGINLIB_EXPORT_CLASS(gazebo_ros2_control::DefaultRobotHWSim, gazebo_ros2_control::RobotHWSim)
