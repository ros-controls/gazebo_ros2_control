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

#include <memory>
#include <string>
#include <vector>

#include "gazebo_ros2_control/gazebo_system.hpp"

namespace gazebo_ros2_control
{

bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model * const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions,
  sdf::ElementPtr sdf)
{
  last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->parent_model_ = parent_model;
  n_dof_ = transmissions.size();

  joint_names_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_pos_state_.resize(n_dof_);
  joint_vel_state_.resize(n_dof_);
  joint_eff_state_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  last_joint_position_cmd_.resize(n_dof_);
  joint_velocity_cmd_.resize(n_dof_);
  joint_effort_cmd_.resize(n_dof_);
  joint_pos_cmd_.resize(n_dof_);
  joint_vel_cmd_.resize(n_dof_);
  joint_eff_cmd_.resize(n_dof_);

  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  for (unsigned int j = 0; j < this->n_dof_; j++) {
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
      // TODO(anyone): Deprecate HW interface specification in actuators in ROS 2
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
      // only a warning, allow joint to continue
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Joint " << transmissions[j].name <<
          " of transmission " << transmissions[j].name <<
          " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one." <<
          "Using the first entry");
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

    // register the state handles
    joint_pos_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_POSITION, &joint_position_[j]);
    joint_vel_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[j]);
    joint_eff_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_EFFORT, &joint_effort_[j]);

    // register the state handles
    joint_pos_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_POSITION, &joint_position_cmd_[j]);
    joint_vel_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_cmd_[j]);
    joint_eff_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_EFFORT, &joint_effort_cmd_[j]);

    try {
      nh_->declare_parameter(transmissions[j].joints[0].name + ".p");
      nh_->declare_parameter(transmissions[j].joints[0].name + ".i");
      nh_->declare_parameter(transmissions[j].joints[0].name + ".d");
      nh_->declare_parameter(transmissions[j].joints[0].name + ".i_clamp_max");
      nh_->declare_parameter(transmissions[j].joints[0].name + ".i_clamp_min");
      nh_->declare_parameter(transmissions[j].joints[0].name + ".antiwindup", false);

      if (nh_->get_parameter(transmissions[j].joints[0].name + ".p").get_type() ==
        rclcpp::PARAMETER_DOUBLE &&
        nh_->get_parameter(transmissions[j].joints[0].name + ".i").get_type() ==
        rclcpp::PARAMETER_DOUBLE &&
        nh_->get_parameter(transmissions[j].joints[0].name + ".d").get_type() ==
        rclcpp::PARAMETER_DOUBLE)
      {
        pid_controllers_.push_back(
          control_toolbox::PidROS(nh_, transmissions[j].joints[0].name));
        if (pid_controllers_[j].initPid()) {
          switch (joint_control_methods_[j]) {
            case POSITION: joint_control_methods_[j] = POSITION_PID;
              RCLCPP_INFO(
                nh_->get_logger(), "joint %s is configured in POSITION_PID mode",
                transmissions[j].joints[0].name.c_str());
              break;
            case VELOCITY: joint_control_methods_[j] = VELOCITY_PID;
              RCLCPP_INFO(
                nh_->get_logger(), "joint %s is configured in VELOCITY_PID mode",
                transmissions[j].joints[0].name.c_str());
              break;
            case EFFORT:
              {}            // fallthrough
            case POSITION_PID:
              {}             // fallthrough
            case VELOCITY_PID:
              {}           // fallthrough
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(nh_->get_logger(), "%s", e.what());
    }
  }

  // Initialize the emergency stop code.
  this->e_stop_active_ = false;
  if (sdf->HasElement("e_stop_topic")) {
    const std::string e_stop_topic = sdf->GetElement("e_stop_topic")->Get<std::string>();
    rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
    this->e_stop_sub_ = this->nh_->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic,
      qos,
      std::bind(&GazeboSystem::eStopCB, this, std::placeholders::_1));
  }

  return true;
}

hardware_interface::return_type
GazeboSystem::configure(const hardware_interface::HardwareInfo & actuator_info)
{
  if (configure_default(actuator_info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
GazeboSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]));
  }
  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
  }
  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_effort_cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type GazeboSystem::start()
{
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::stop()
{
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::read()
{
  for (unsigned int j = 0; j < this->joint_names_.size(); j++) {
    joint_position_[j] = this->sim_joints_[j]->Position(0);
    joint_velocity_[j] = this->sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = this->sim_joints_[j]->GetForce(0u);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = this->parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  for (unsigned int j = 0; j < this->joint_names_.size(); j++) {
    switch (joint_control_methods_[j]) {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_cmd_[j];
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
      case POSITION:
        if (e_stop_active_) {
          // If the E-stop is active, joints controlled by position commands will maintain
          // their positions.
          this->sim_joints_[j]->SetPosition(0, last_joint_position_cmd_[j], true);
        } else {
          this->sim_joints_[j]->SetPosition(0, joint_position_cmd_[j], true);
          last_joint_position_cmd_[j] = joint_position_cmd_[j];
        }
        break;
      case POSITION_PID:
        {
          double error;
          // TODO(ahcorde): Restore this part
          // switch (joint_types_[j]) {
          // case urdf::Joint::REVOLUTE:
          //   // angles::shortest_angular_distance_with_limits(
          //   //   joint_pos_state_[j].get_value(),
          //   //   joint_pos_cmdh_[j].get_value(),
          //   //   joint_lower_limits_[j],
          //   //   joint_upper_limits_[j],
          //   //   error);
          //   break;
          // case urdf::Joint::CONTINUOUS:
          //   error = angles::shortest_angular_distance(
          //     joint_position_[j],
          //     joint_position_cmd_[j]);
          //   break;
          // default:
          // error = joint_position_cmd_[j] - joint_position_[j];
          // }
          error = joint_position_cmd_[j] - joint_position_[j];

          // TODO(ahcorde): Restore this when Jonit_limit interface is available
          // const double effort_limit = joint_effort_limits_[j];
          // const double effort = ignition::math::clamp(
          //   pid_controllers_[j].computeCommand(error, period),
          //   -effort_limit, effort_limit);

          const double effort = pid_controllers_[j].computeCommand(error, sim_period);

          sim_joints_[j]->SetForce(0, effort);
          sim_joints_[j]->SetParam("friction", 0, 0.0);
        }
        break;
      case VELOCITY:
        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_cmd_[j]);
        break;
      case VELOCITY_PID:
        double error;
        if (e_stop_active_) {
          error = -joint_velocity_[j];
        } else {
          error = joint_velocity_cmd_[j] - joint_velocity_[j];
        }
        // TODO(ahcorde): Restore this when Jonit_limit interface is available
        // const double effort_limit = joint_effort_limits_[j];
        // const double effort = ignition::math::clamp(
        //   pid_controllers_[j].computeCommand(error, period),
        //   -effort_limit, effort_limit);
        const double effort = pid_controllers_[j].computeCommand(error, sim_period);
        sim_joints_[j]->SetForce(0, effort);
        break;
    }
  }

  last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}

// Emergency stop callback
void GazeboSystem::eStopCB(const std::shared_ptr<std_msgs::msg::Bool> e_stop_active)
{
  this->e_stop_active_ = e_stop_active->data;
}

}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
