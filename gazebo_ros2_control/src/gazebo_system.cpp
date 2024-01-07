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

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gazebo_ros2_control/gazebo_system.hpp"
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"
#include "gazebo/sensors/SensorManager.hh"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
};

class gazebo_ros2_control::GazeboSystemPrivate
{
public:
  GazeboSystemPrivate() = default;

  ~GazeboSystemPrivate() = default;

  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief Number of sensors.
  size_t n_sensors_;

  /// \brief Gazebo Model Ptr.
  gazebo::physics::ModelPtr parent_model_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<GazeboSystemInterface::ControlMethod> joint_control_methods_;

  /// \brief vector with indication for actuated joints (vs. passive joints)
  std::vector<bool> is_joint_actuated_;

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

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief handles to the imus from within Gazebo
  std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::vector<std::array<double, 10>> imu_sensor_data_;

  /// \brief handles to the FT sensors from within Gazebo
  std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

  /// \brief An array per FT sensor for 3D force and torquee
  std::vector<std::array<double, 6>> ft_sensor_data_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  // Should hold the joints if no control_mode is active
  bool hold_joints_ = true;
};

namespace gazebo_ros2_control
{

bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  this->dataPtr = std::make_unique<GazeboSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->parent_model_ = parent_model;

  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  try {
    this->dataPtr->hold_joints_ = this->nh_->get_parameter("hold_joints").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  }
  RCLCPP_DEBUG_STREAM(
    this->nh_->get_logger(), "hold_joints (system): " << this->dataPtr->hold_joints_ << std::endl);

  registerJoints(hardware_info, parent_model);
  registerSensors(hardware_info, parent_model);

  if (this->dataPtr->n_dof_ == 0 && this->dataPtr->n_sensors_ == 0) {
    RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is no joint or sensor available");
    return false;
  }

  return true;
}

void GazeboSystem::registerJoints(
  const hardware_interface::HardwareInfo & hardware_info,
  gazebo::physics::ModelPtr parent_model)
{
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
  this->dataPtr->is_joint_actuated_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_dof_);

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joint_names_[j] = joint_info.name;

    gazebo::physics::JointPtr simjoint = parent_model->GetJoint(joint_name);
    this->dataPtr->sim_joints_.push_back(simjoint);
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    std::string suffix = "";

    // check if joint is mimicked
    if (joint_info.parameters.find("mimic") != joint_info.parameters.end()) {
      const auto mimicked_joint = joint_info.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo & info) {
          return info.name == mimicked_joint;
        });
      if (mimicked_joint_it == hardware_info.joints.end()) {
        throw std::runtime_error(
                std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_index = j;
      mimic_joint.mimicked_joint_index = std::distance(
        hardware_info.joints.begin(), mimicked_joint_it);
      auto param_it = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end()) {
        mimic_joint.multiplier = hardware_interface::stod(joint_info.parameters.at("multiplier"));
      } else {
        mimic_joint.multiplier = 1.0;
      }
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '" << mimicked_joint <<
          "' with mutiplier: " << mimic_joint.multiplier);
      this->dataPtr->mimic_joints_.push_back(mimic_joint);
      suffix = "_mimic";
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo & interface_info) {
        if (!interface_info.initial_value.empty()) {
          double value = hardware_interface::stod(interface_info.initial_value);
          RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
          return value;
        } else {
          return 0.0;
        }
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); i++) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joint_position_[j]);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joint_position_[j] = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joint_velocity_[j]);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joint_velocity_[j] = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joint_effort_[j]);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joint_effort_[j] = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); i++) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joint_position_cmd_[j]);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joint_position_cmd_[j] = initial_position;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->sim_joints_[j]->SetPosition(0, initial_position, true);
      }
      if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joint_velocity_cmd_[j]);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joint_velocity_cmd_[j] = initial_velocity;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->sim_joints_[j]->SetVelocity(0, initial_velocity);
      }
      if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joint_effort_cmd_[j]);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joint_effort_cmd_[j] = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_effort)) {
        this->dataPtr->sim_joints_[j]->SetForce(0, initial_effort);
      }
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->is_joint_actuated_[j] = (joint_info.command_interfaces.size() > 0);
  }
}

void GazeboSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info,
  gazebo::physics::ModelPtr parent_model)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> imu_components_;
  std::vector<hardware_interface::ComponentInfo> ft_sensor_components_;

  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures
  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    std::string sensor_name = component.name;

    // This can't be used, because it only looks for sensor in links, but force_torque_sensor
    // must be in a joint, as in not found by SensorScopedName
    // std::vector<std::string> gz_sensor_names = parent_model->SensorScopedName(sensor_name);

    // Workaround to find sensors whose parent is a link or joint of parent_model
    std::vector<std::string> gz_sensor_names;
    for (const auto & s : gazebo::sensors::SensorManager::Instance()->GetSensors()) {
      const std::string sensor_parent = s->ParentName();
      if (s->Name() != sensor_name) {
        continue;
      }
      if ((parent_model->GetJoint(sensor_parent) != nullptr) ||
        parent_model->GetLink(sensor_parent) != nullptr)
      {
        gz_sensor_names.push_back(s->ScopedName());
      }
    }
    if (gz_sensor_names.empty()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping sensor in the URDF named '" << sensor_name <<
          "' which is not in the gazebo model.");
      continue;
    }
    if (gz_sensor_names.size() > 1) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Sensor in the URDF named '" << sensor_name <<
          "' has more than one gazebo sensor with the " <<
          "same name, only using the first. It has " << gz_sensor_names.size() << " sensors");
    }

    gazebo::sensors::SensorPtr simsensor = gazebo::sensors::SensorManager::Instance()->GetSensor(
      gz_sensor_names[0]);
    if (!simsensor) {
      RCLCPP_ERROR_STREAM(
        this->nh_->get_logger(),
        "Error retrieving sensor '" << sensor_name << " from the sensor manager");
      continue;
    }
    if (simsensor->Type() == "imu") {
      gazebo::sensors::ImuSensorPtr imu_sensor =
        std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(simsensor);
      if (!imu_sensor) {
        RCLCPP_ERROR_STREAM(
          this->nh_->get_logger(),
          "Error retrieving casting sensor '" << sensor_name << " to ImuSensor");
        continue;
      }
      imu_components_.push_back(component);
      this->dataPtr->sim_imu_sensors_.push_back(imu_sensor);
    } else if (simsensor->Type() == "force_torque") {
      gazebo::sensors::ForceTorqueSensorPtr ft_sensor =
        std::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(simsensor);
      if (!ft_sensor) {
        RCLCPP_ERROR_STREAM(
          this->nh_->get_logger(),
          "Error retrieving casting sensor '" << sensor_name << " to ForceTorqueSensor");
        continue;
      }
      ft_sensor_components_.push_back(component);
      this->dataPtr->sim_ft_sensors_.push_back(ft_sensor);
    }
  }

  this->dataPtr->imu_sensor_data_.resize(this->dataPtr->sim_imu_sensors_.size());
  this->dataPtr->ft_sensor_data_.resize(this->dataPtr->sim_ft_sensors_.size());
  this->dataPtr->n_sensors_ = this->dataPtr->sim_imu_sensors_.size() +
    this->dataPtr->sim_ft_sensors_.size();

  for (unsigned int i = 0; i < imu_components_.size(); i++) {
    const std::string & sensor_name = imu_components_[i].name;
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << sensor_name);
    RCLCPP_INFO_STREAM(
      this->nh_->get_logger(), "\tState:");
    for (const auto & state_interface : imu_components_[i].state_interfaces) {
      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

      size_t data_index = interface_name_map.at(state_interface.name);
      this->dataPtr->state_interfaces_.emplace_back(
        sensor_name,
        state_interface.name,
        &this->dataPtr->imu_sensor_data_[i][data_index]);
    }
  }
  for (unsigned int i = 0; i < ft_sensor_components_.size(); i++) {
    const std::string & sensor_name = ft_sensor_components_[i].name;
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << sensor_name);
    RCLCPP_INFO_STREAM(
      this->nh_->get_logger(), "\tState:");
    for (const auto & state_interface : ft_sensor_components_[i].state_interfaces) {
      static const std::map<std::string, size_t> interface_name_map = {
        {"force.x", 0},
        {"force.y", 1},
        {"force.z", 2},
        {"torque.x", 3},
        {"torque.y", 4},
        {"torque.z", 5}
      };
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

      size_t data_index = interface_name_map.at(state_interface.name);
      this->dataPtr->state_interfaces_.emplace_back(
        sensor_name,
        state_interface.name,
        &this->dataPtr->ft_sensor_data_[i][data_index]);
    }
  }
}

CallbackReturn
GazeboSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GazeboSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
GazeboSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    for (const std::string & interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name == (this->dataPtr->joint_names_[j] + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joint_control_methods_[j] &= static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joint_control_methods_[j] &= static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joint_control_methods_[j] &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {
      if (interface_name == (this->dataPtr->joint_names_[j] + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joint_control_methods_[j] |= POSITION;
      } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joint_control_methods_[j] |= VELOCITY;
      } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joint_control_methods_[j] |= EFFORT;
      }
    }
  }

  // mimic joint has the same control mode as mimicked joint
  for (const auto & mimic_joint : this->dataPtr->mimic_joints_) {
    this->dataPtr->joint_control_methods_[mimic_joint.joint_index] =
      this->dataPtr->joint_control_methods_[mimic_joint.mimicked_joint_index];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->Position(0);
      this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
      this->dataPtr->joint_effort_[j] = this->dataPtr->sim_joints_[j]->GetForce(0u);
    }
  }

  for (unsigned int j = 0; j < this->dataPtr->sim_imu_sensors_.size(); j++) {
    auto sim_imu = this->dataPtr->sim_imu_sensors_[j];
    this->dataPtr->imu_sensor_data_[j][0] = sim_imu->Orientation().X();
    this->dataPtr->imu_sensor_data_[j][1] = sim_imu->Orientation().Y();
    this->dataPtr->imu_sensor_data_[j][2] = sim_imu->Orientation().Z();
    this->dataPtr->imu_sensor_data_[j][3] = sim_imu->Orientation().W();

    this->dataPtr->imu_sensor_data_[j][4] = sim_imu->AngularVelocity().X();
    this->dataPtr->imu_sensor_data_[j][5] = sim_imu->AngularVelocity().Y();
    this->dataPtr->imu_sensor_data_[j][6] = sim_imu->AngularVelocity().Z();

    this->dataPtr->imu_sensor_data_[j][7] = sim_imu->LinearAcceleration().X();
    this->dataPtr->imu_sensor_data_[j][8] = sim_imu->LinearAcceleration().Y();
    this->dataPtr->imu_sensor_data_[j][9] = sim_imu->LinearAcceleration().Z();
  }

  for (unsigned int j = 0; j < this->dataPtr->sim_ft_sensors_.size(); j++) {
    auto sim_ft_sensor = this->dataPtr->sim_ft_sensors_[j];
    this->dataPtr->imu_sensor_data_[j][0] = sim_ft_sensor->Force().X();
    this->dataPtr->imu_sensor_data_[j][1] = sim_ft_sensor->Force().Y();
    this->dataPtr->imu_sensor_data_[j][2] = sim_ft_sensor->Force().Z();

    this->dataPtr->imu_sensor_data_[j][3] = sim_ft_sensor->Torque().X();
    this->dataPtr->imu_sensor_data_[j][4] = sim_ft_sensor->Torque().Y();
    this->dataPtr->imu_sensor_data_[j][5] = sim_ft_sensor->Torque().Z();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = this->dataPtr->parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  // set values of all mimic joints with respect to mimicked joint
  for (const auto & mimic_joint : this->dataPtr->mimic_joints_) {
    if (this->dataPtr->joint_control_methods_[mimic_joint.joint_index] & POSITION &&
      this->dataPtr->joint_control_methods_[mimic_joint.mimicked_joint_index] & POSITION)
    {
      this->dataPtr->joint_position_cmd_[mimic_joint.joint_index] =
        mimic_joint.multiplier *
        this->dataPtr->joint_position_cmd_[mimic_joint.mimicked_joint_index];
    } else if (this->dataPtr->joint_control_methods_[mimic_joint.joint_index] & VELOCITY && // NOLINT
      this->dataPtr->joint_control_methods_[mimic_joint.mimicked_joint_index] & VELOCITY)
    {
      this->dataPtr->joint_velocity_cmd_[mimic_joint.joint_index] =
        mimic_joint.multiplier *
        this->dataPtr->joint_velocity_cmd_[mimic_joint.mimicked_joint_index];
    } else if (this->dataPtr->joint_control_methods_[mimic_joint.joint_index] & EFFORT && // NOLINT
      this->dataPtr->joint_control_methods_[mimic_joint.mimicked_joint_index] & EFFORT)
    {
      this->dataPtr->joint_effort_cmd_[mimic_joint.joint_index] =
        mimic_joint.multiplier * this->dataPtr->joint_effort_cmd_[mimic_joint.mimicked_joint_index];
    }
  }

  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      if (this->dataPtr->joint_control_methods_[j] & POSITION) {
        this->dataPtr->sim_joints_[j]->SetPosition(0, this->dataPtr->joint_position_cmd_[j], true);
        this->dataPtr->sim_joints_[j]->SetVelocity(0, 0.0);
      } else if (this->dataPtr->joint_control_methods_[j] & VELOCITY) { // NOLINT
        this->dataPtr->sim_joints_[j]->SetVelocity(0, this->dataPtr->joint_velocity_cmd_[j]);
      } else if (this->dataPtr->joint_control_methods_[j] & EFFORT) { // NOLINT
        this->dataPtr->sim_joints_[j]->SetForce(0, this->dataPtr->joint_effort_cmd_[j]);
      } else if (this->dataPtr->is_joint_actuated_[j] && this->dataPtr->hold_joints_) {
        // Fallback case is a velocity command of zero (only for actuated joints)
        this->dataPtr->sim_joints_[j]->SetVelocity(0, 0.0);
      }
    }
  }

  this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}
}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
