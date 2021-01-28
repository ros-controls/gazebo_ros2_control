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

class gazebo_ros2_control::GazeboSystemPrivate
{
public:
  GazeboSystemPrivate() = default;

  ~GazeboSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

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
  std::vector<GazeboSystemInterface::ControlMethod> joint_control_methods_;

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

  /// \brief callback to get the e_stop signal from the ROS 2 topic
  void eStopCB(const std::shared_ptr<std_msgs::msg::Bool> e_stop_active)
  {
    this->e_stop_active_ = e_stop_active->data;
  }
};

namespace gazebo_ros2_control
{

bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions,
  sdf::ElementPtr sdf)
{
  this->dataPtr = std::make_unique<GazeboSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->parent_model_ = parent_model;
  this->dataPtr->n_dof_ = transmissions.size();

  this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_pos_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_vel_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_eff_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->last_joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_pos_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_vel_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_eff_cmd_.resize(this->dataPtr->n_dof_);

  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    //
    // Perform some validation on the URDF joint and actuator spec
    //
    // Check that this transmission has one joint
    if (transmissions[j].joints.empty()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Transmission " << transmissions[j].name <<
          " has no associated joints.");
      continue;
    } else if (transmissions[j].joints.size() > 1) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Transmission " << transmissions[j].name <<
          " has more than one joint. Currently the default robot hardware simulation " <<
          " interface only supports one.");
      continue;
    }
    std::string joint_name = this->dataPtr->joint_names_[j] = transmissions[j].joints[0].name;

    std::vector<std::string> joint_interfaces = transmissions[j].joints[0].interfaces;
    if (joint_interfaces.empty() &&
      !(transmissions[j].actuators.empty()) &&
      !(transmissions[j].actuators[0].interfaces.empty()))
    {
      // TODO(anyone): Deprecate HW interface specification in actuators in ROS 2
      joint_interfaces = transmissions[j].actuators[0].interfaces;
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "The <hardware_interface> element of transmission " <<
          transmissions[j].name << " should be nested inside the <joint> element," <<
          " not <actuator>. The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Joint " << transmissions[j].name <<
          " of transmission " << transmissions[j].name <<
          " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
      continue;
    } else if (joint_interfaces.size() > 1) {
      // only a warning, allow joint to continue
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Joint " << transmissions[j].name <<
          " of transmission " << transmissions[j].name <<
          " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one." <<
          "Using the first entry");
    }
    std::string hardware_interface = joint_interfaces.front();
    // Decide what kind of command interface this actuator/joint has
    if (hardware_interface == "hardware_interface/EffortJointInterface") {
      this->dataPtr->joint_control_methods_[j] = EFFORT;
    } else if (hardware_interface == "hardware_interface/PositionJointInterface") {
      this->dataPtr->joint_control_methods_[j] = POSITION;
    } else if (hardware_interface == "hardware_interface/VelocityJointInterface") {
      this->dataPtr->joint_control_methods_[j] = VELOCITY;
    } else {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "No matching joint interface '" <<
          hardware_interface << "' for joint " << joint_name);
      RCLCPP_INFO(
        this->nh_->get_logger(),
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
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }
    this->dataPtr->sim_joints_.push_back(simjoint);

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(
      this->nh_->get_logger(), "Loading joint '" << joint_name << "' of type '" <<
        hardware_interface << "'");

    // register the state handles
    this->dataPtr->joint_pos_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_[j]);
    this->dataPtr->joint_vel_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_[j]);
    this->dataPtr->joint_eff_state_[j] = std::make_shared<hardware_interface::StateInterface>(
      joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_[j]);

    // register the state handles
    this->dataPtr->joint_pos_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_cmd_[j]);
    this->dataPtr->joint_vel_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_cmd_[j]);
    this->dataPtr->joint_eff_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
      joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_cmd_[j]);
  }

  // Initialize the emergency stop code.
  this->dataPtr->e_stop_active_ = false;
  if (sdf->HasElement("e_stop_topic")) {
    const std::string e_stop_topic = sdf->GetElement("e_stop_topic")->Get<std::string>();
    rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
    this->dataPtr->e_stop_sub_ = this->nh_->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic,
      qos,
      std::bind(&GazeboSystemPrivate::eStopCB, this->dataPtr.get(), std::placeholders::_1));
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

  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joint_position_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joint_velocity_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joint_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joint_position_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joint_velocity_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joint_effort_cmd_[i]));
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
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->Position(0);
    this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
    this->dataPtr->joint_effort_[j] = this->dataPtr->sim_joints_[j]->GetForce(0u);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = this->dataPtr->parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    switch (this->dataPtr->joint_control_methods_[j]) {
      case EFFORT:
        {
          const double effort =
            this->dataPtr->e_stop_active_ ? 0 : this->dataPtr->joint_effort_cmd_[j];
          this->dataPtr->sim_joints_[j]->SetForce(0, effort);
        }
        break;
      case POSITION:
        if (this->dataPtr->e_stop_active_) {
          // If the E-stop is active, joints controlled by position commands will maintain
          // their positions.
          this->dataPtr->sim_joints_[j]->SetPosition(
            0, this->dataPtr->last_joint_position_cmd_[j],
            true);
        } else {
          this->dataPtr->sim_joints_[j]->SetPosition(
            0, this->dataPtr->joint_position_cmd_[j],
            true);
          this->dataPtr->last_joint_position_cmd_[j] = this->dataPtr->joint_position_cmd_[j];
        }
        break;
      case VELOCITY:
        this->dataPtr->sim_joints_[j]->SetVelocity(
          0,
          this->dataPtr->e_stop_active_ ? 0 : this->dataPtr->joint_velocity_cmd_[j]);
        break;
    }
  }

  this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}
}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
