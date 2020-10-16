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


#include <gazebo_ros2_control/default_robot_hw_sim.h>
#include <urdf/model.h>

namespace gazebo_ros2_control
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("default_robot_hw_sim");

bool DefaultRobotHWSim::initSim(
  const std::string& robot_namespace,
  rclcpp::Node::SharedPtr& model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  std::cerr << "DefaultRobotHWSim::InitSim " << std::endl;

  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  rclcpp::Node::SharedPtr& joint_limit_nh = model_nh;

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
  joint_opmodes_.resize(n_dof_);

  joint_pos_stateh_.resize(n_dof_);
  joint_vel_stateh_.resize(n_dof_);
  joint_eff_stateh_.resize(n_dof_);
  joint_pos_cmdh_.resize(n_dof_);
  joint_opmodehandles_.resize(n_dof_);
  joint_eff_cmdh_.resize(n_dof_);
  joint_vel_cmdh_.resize(n_dof_);

  joint_limit_handles_.resize(n_dof_);
  joint_soft_limit_handles_.resize(n_dof_);

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints.empty())
    {
      std::cerr << "Transmission " << transmissions[j].name
        << " has no associated joints.";
      continue;
    }
    else if(transmissions[j].joints.size() > 1)
    {
      std::cerr << "Transmission " << transmissions[j].name
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.";
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints[0].hardware_interfaces;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators.empty()) &&
        !(transmissions[j].actuators[0].hardware_interfaces.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators[0].hardware_interfaces;
      std::cerr << "The <hardware_interface> element of transmission " <<
        transmissions[j].name << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.";
    }
    if (joint_interfaces.empty())
    {
      std::cerr << "Joint " << transmissions[j].name <<
        " of transmission " << transmissions[j].name << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.";
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      std::cerr << "Joint " << transmissions[j].name <<
        " of transmission " << transmissions[j].name << " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. Using the first entry";
      //continue;
    }

    // Add data from transmission
    std::string joint_name = joint_names_[j] = transmissions[j].joints[0].name;

    const std::string& hardware_interface = joint_interfaces.front();

    // Debug
    std::cerr << "Loading joint '" << joint_name
      << "' of type '" << hardware_interface << "'" << std::endl;

    // Create joint state interface for all joints
    if(
          register_joint(joint_name, "position", 1.0) != hardware_interface::return_type::OK ||
          register_joint(joint_name, "velocity", 0.0) != hardware_interface::return_type::OK ||
          register_joint(joint_name, "effort", 1.0) != hardware_interface::return_type::OK) {
          std::cerr << "Failed to register position, velocity and effort state handles for joint " << joint_name;
          return false;
    }

    // get the joint state handles we just registered
    // todo: put this above into the error checking...maybe do a loop instead?
    joint_pos_stateh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "position");
    joint_vel_stateh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "velocity");
    joint_eff_stateh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "effort");
    get_joint_handle(*joint_pos_stateh_[j]);
    get_joint_handle(*joint_vel_stateh_[j]);
    get_joint_handle(*joint_eff_stateh_[j]);

    // Decide what kind of command interface this actuator/joint has
    //hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      register_joint(joint_name, "effort_command", 0.0);
      joint_eff_cmdh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "effort_command");
      get_joint_handle(*joint_eff_cmdh_[j]);

#if 0 //@todo
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);

      ej_interface_.registerHandle(joint_handle);
#endif
    }
    else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      register_joint(joint_name, "position_command", 0.0);
      joint_pos_cmdh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "position_command");
      get_joint_handle(*joint_pos_cmdh_[j]);
#if 0 //@todo
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);
#endif
    }
    else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
    {
      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      register_joint(joint_name, "velocity_command", 0.0);
      joint_vel_cmdh_[j] = std::make_shared<hardware_interface::JointHandle>(joint_name, "velocity_command");
      get_joint_handle(*joint_vel_cmdh_[j]);

#if 0 //@todo
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);
#endif
    }
    else
    {
      std::cerr << "No matching hardware interface found for '"
        << hardware_interface << "' while loading interfaces for " << joint_name << std::endl;
      return false;
    }
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
      std::cerr << "Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_name << "'." << std::endl;
    }

    // Get the gazebo joint that corresponds to the robot joint.
    //RCLCPP_DEBUG_STREAM(LOGGER, "Getting pointer to gazebo joint: "
    //  << joint_name);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name);
    if (!joint)
    {
      std::cerr << "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model." << std::endl;
      return false;
    }
    sim_joints_.push_back(joint);
    joint_pos_stateh_[j]->set_value(joint->Position(0) );
    joint_vel_stateh_[j]->set_value(joint->GetVelocity(0) );
    joint_eff_stateh_[j]->set_value(joint->GetForce(0) );

    // get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
    physics_type_ = physics->GetType();
    if (physics_type_.empty())
    {
      std::cerr << "No physics type found." << std::endl;
    }

    registerJointLimits(j,
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j], &joint_vel_limits_[j]);
    if (joint_control_methods_[j] != EFFORT)
    {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetParam("vel") to control the joint.
#if 0
      const ros::NodeHandle nh(robot_namespace + "//pid_gains/" +
                               joint_names_[j]);
#endif
#if 0
      if (pid_controllers_[j].init(nh))
      {
        switch (joint_control_methods_[j])
        {
          case POSITION:
            joint_control_methods_[j] = POSITION_PID;
            break;
          case VELOCITY:
            joint_control_methods_[j] = VELOCITY_PID;
            break;
        }
      }
      else
#endif
      {
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
#if GAZEBO_MAJOR_VERSION > 2
        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
#else
        joint->SetMaxForce(0, joint_effort_limits_[j]);
#endif
      }

    }


    joint_opmodes_[j] = hardware_interface::OperationMode::ACTIVE;
    joint_opmodehandles_[j] = hardware_interface::OperationModeHandle(
      joint_name, &joint_opmodes_[j]);
    if (register_operation_mode_handle(&joint_opmodehandles_[j]) != hardware_interface::return_type::OK) {
      RCLCPP_WARN_ONCE(LOGGER,"cant register jointopmodehandle");
    }

#if 0 //@note (guru-florida): dead code. This sets limits then doesnt do anything with the object
    joint_limits_interface::JointLimits limits; //hack, refactor registerjointhandle
    limits.has_position_limits = true;
    limits.min_position = joint_lower_limits_[j];
    limits.max_position = joint_upper_limits_[j];
    limits.max_velocity = joint_vel_limits_[j];
    limits.has_velocity_limits = true;
    limits.max_effort = joint_effort_limits_[j];
    limits.has_effort_limits = true;
#endif
  }
#if 0 //@todo
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
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
    double position = sim_joints_[j]->Position(0);
#else
    double position = sim_joints_[j]->GetAngle(0).Radian();
#endif
    if (joint_types_[j] == urdf::Joint::PRISMATIC)
    {
      joint_pos_stateh_[j]->set_value(position);
    }
    else
    {
      double prev_position = joint_pos_stateh_[j]->get_value();
      joint_pos_stateh_[j]->set_value(prev_position + angles::shortest_angular_distance(prev_position, position));
    }
    joint_vel_stateh_[j]->set_value(sim_joints_[j]->GetVelocity(0));
    joint_eff_stateh_[j]->set_value(sim_joints_[j]->GetForce((unsigned int)(0)));
  }
}

void DefaultRobotHWSim::writeSim(rclcpp::Time time, rclcpp::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_.clear();
      std::transform(joint_pos_stateh_.begin(), joint_pos_stateh_.end(),
                     std::back_inserter(last_joint_position_command_),
                     [](const std::shared_ptr<hardware_interface::JointHandle>& ph) { return  ph->get_value(); });
      last_e_stop_active_ = true;
    }
    for(int i=0; i < n_dof_; i++) {
      joint_pos_cmdh_[i]->set_value(last_joint_position_command_[i]);
    }
  }
  else
  {
    last_e_stop_active_ = false;
  }
#if 0 //@todo
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
#else
  for (auto limit_handle : joint_limit_handles_)
  {
    limit_handle->enforceLimits(period);
  }
  for (auto limit_handle : joint_soft_limit_handles_)
  {
    limit_handle->enforceLimits(period);
  }

#endif
  for(unsigned int j=0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_eff_cmdh_[j]->get_value();
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
#if GAZEBO_MAJOR_VERSION >= 9
        sim_joints_[j]->SetPosition(0, joint_pos_cmdh_[j]->get_value(), true);
#else
        RCLCPP_WARN_ONCE(LOGGER,"The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.");
        RCLCPP_WARN_ONCE(LOGGER,"As a result, gravity will not be simulated correctly for your model.");
        RCLCPP_WARN_ONCE(LOGGER,"Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.");
        RCLCPP_WARN_ONCE(LOGGER,"For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
        sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#endif
        break;

      case POSITION_PID:
        {
          double error;
          switch (joint_types_[j])
          {
            case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(joint_pos_stateh_[j]->get_value(),
                                                            joint_pos_cmdh_[j]->get_value(),
                                                            joint_lower_limits_[j],
                                                            joint_upper_limits_[j],
                                                            error);
              break;
            case urdf::Joint::CONTINUOUS:
              error = angles::shortest_angular_distance(joint_pos_stateh_[j]->get_value(),
                                                        joint_pos_cmdh_[j]->get_value());
              break;
            default:
              error = joint_pos_cmdh_[j]->get_value() - joint_pos_stateh_[j]->get_value();
          }

          const double effort_limit = joint_effort_limits_[j];
          //TODO(anyone): Restored this when PID controllers is available
          // const double effort = std::clamp(pid_controllers_[j].computeCommand(error, period),
          //                             -effort_limit, effort_limit);
          const double effort = 0.0;
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case VELOCITY:
#if GAZEBO_MAJOR_VERSION > 2
        if (physics_type_.compare("ode") == 0)
        {
          sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_vel_cmdh_[j]->get_value());
        }
        else
        {
          sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_vel_cmdh_[j]->get_value());
        }
#else
        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
#endif
        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_)
          error = -joint_vel_stateh_[j]->get_value();
        else
          error = joint_vel_cmdh_[j]->get_value() - joint_vel_stateh_[j]->get_value();
        const double effort_limit = joint_effort_limits_[j];
        //TODO(anyone): Restored this when PID controllers is available
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
                         size_t joint_nr,
                         const rclcpp::Node::SharedPtr& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit,
                         double *const vel_limit)
{
  const std::string& joint_name = joint_names_[joint_nr];
  const ControlMethod ctrl_method = joint_control_methods_[joint_nr];

  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;

  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint->name, joint_limit_nh, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint->name, joint_limit_nh, soft_limits))
        has_soft_limits = true;

      //@note (ddeng): these joint limits arent input into the node, so fetch them from the urdf
      //@todo (guru-florida): These are now a part of the node as of 2020-09-03, so what one do we take?
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

      //urdf_joint->safety->k_position;
      has_limits = true;
    }
  }

  // Get limits from the parameter server.
  //@note: no longer using parameter servers
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          joint_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::EffortJointSaturationHandle>(
              joint_pos_cmdh_[joint_nr], joint_vel_cmdh_[joint_nr], joint_eff_cmdh_[joint_nr], limits);
        }
        break;
      case POSITION:
        {
          joint_soft_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::PositionJointSoftLimitsHandle>(
              joint_pos_stateh_[joint_nr], joint_pos_cmdh_[joint_nr], limits, soft_limits);
        }
        break;
      case VELOCITY:
        {
          joint_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::VelocityJointSaturationHandle>(
              joint_vel_stateh_[joint_nr], joint_vel_cmdh_[joint_nr], limits);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          joint_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::EffortJointSaturationHandle>(
              joint_pos_cmdh_[joint_nr], joint_vel_cmdh_[joint_nr], joint_eff_cmdh_[joint_nr], limits);
        }
        break;
      case POSITION:
        {
          joint_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::PositionJointSaturationHandle>(
              joint_pos_stateh_[joint_nr], joint_pos_cmdh_[joint_nr], limits);
        }
        break;
      case VELOCITY:
        {
          joint_limit_handles_[joint_nr] = std::make_shared<joint_limits_interface::VelocityJointSaturationHandle>(
              joint_vel_stateh_[joint_nr], joint_vel_cmdh_[joint_nr], limits);
        }
        break;
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros2_control::DefaultRobotHWSim, gazebo_ros2_control::RobotHWSim)
