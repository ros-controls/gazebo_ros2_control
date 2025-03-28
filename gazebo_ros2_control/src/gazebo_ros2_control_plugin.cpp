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
   Desc:   Gazebo plugin for ros2_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>

#include "gazebo_ros/node.hpp"

#include "gazebo_ros2_control/gazebo_ros2_control_plugin.hpp"
#include "gazebo_ros2_control/gazebo_system.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace gazebo_ros2_control
{
class GazeboResourceManager : public hardware_interface::ResourceManager
{
public:
  GazeboResourceManager(
    rclcpp::Node::SharedPtr & node,
    gazebo::physics::ModelPtr parent_model,
    sdf::ElementPtr sdf)
  : hardware_interface::ResourceManager(
      node->get_node_clock_interface(), node->get_node_logging_interface()),
    gazebo_system_loader_("gazebo_ros2_control",
      "gazebo_ros2_control::GazeboSystemInterface"),
    logger_(node->get_logger().get_child("GazeboResourceManager"))
  {
    node_ = node;
    parent_model_ = parent_model;
    sdf_ = sdf;
  }

  GazeboResourceManager(const GazeboResourceManager &) = delete;

  // Called from Controller Manager when robot description is initialized from callback
  bool load_and_initialize_components(
    const std::string & urdf,
    unsigned int update_rate) override
  {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

    for (const auto & individual_hardware_info : hardware_info) {
      std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
      RCLCPP_DEBUG(
        logger_, "Load hardware interface %s ...",
        robot_hw_sim_type_str_.c_str());

      // Load hardware
      std::unique_ptr<gazebo_ros2_control::GazeboSystemInterface> gazeboSystem;
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
      try {
        gazeboSystem = std::unique_ptr<gazebo_ros2_control::GazeboSystemInterface>(
          gazebo_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(
          logger_,
          "The plugin failed to load for some reason. Error: %s\n",
          ex.what());
        continue;
      }

      // initialize simulation requirements
      if (!gazeboSystem->initSim(
          node_,
          parent_model_,
          individual_hardware_info,
          sdf_))
      {
        RCLCPP_FATAL(
          logger_, "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(
        logger_, "Initialized robot simulation interface %s!",
        robot_hw_sim_type_str_.c_str());

      // initialize hardware
      import_component(std::move(gazeboSystem), individual_hardware_info);
    }

    return components_are_loaded_and_initialized_;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;

  /// \brief Interface loader
  pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface> gazebo_system_loader_;

  rclcpp::Logger logger_;
};
class GazeboRosControlPrivate
{
public:
  GazeboRosControlPrivate() = default;

  virtual ~GazeboRosControlPrivate() = default;

  // Called by the world update start event
  void Update();

  // Called on world reset
  virtual void Reset();

  // Node Handles
  gazebo_ros::Node::SharedPtr model_nh_;

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<
      gazebo_ros2_control::GazeboSystemInterface>> robot_hw_sim_loader_;

  // Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  // Thread where the executor will sping
  std::thread thread_executor_spin_;

  // Flag to stop the executor thread when this plugin is exiting
  bool stop_;

  // Controller manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Available controllers
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>> controllers_;

  // Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  // Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
};

GazeboRosControlPlugin::GazeboRosControlPlugin()
: impl_(std::make_unique<GazeboRosControlPrivate>())
{
}

GazeboRosControlPlugin::~GazeboRosControlPlugin()
{
  // Stop controller manager thread
  impl_->stop_ = true;
  impl_->executor_->remove_node(impl_->controller_manager_);
  impl_->executor_->cancel();
  impl_->thread_executor_spin_.join();

  // Disconnect from gazebo events
  impl_->update_connection_.reset();
}

// Overloaded Gazebo entry point
void GazeboRosControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("gazebo_ros2_control"),
    "Loading gazebo_ros2_control plugin");

  // Save pointers to the model
  impl_->parent_model_ = parent;

  // Get parameters/settings for controllers from ROS param server
  // Initialize ROS node
  impl_->model_nh_ = gazebo_ros::Node::Get(sdf);

  RCLCPP_INFO(
    impl_->model_nh_->get_logger(), "Starting gazebo_ros2_control plugin in namespace: %s",
    impl_->model_nh_->get_namespace());

  RCLCPP_INFO(
    impl_->model_nh_->get_logger(), "Starting gazebo_ros2_control plugin in ROS 2 node: %s",
    impl_->model_nh_->get_name());

  // Error message if the model couldn't be found
  if (!impl_->parent_model_) {
    RCLCPP_ERROR_STREAM(impl_->model_nh_->get_logger(), "parent model is NULL");
    return;
  }

  // Check that ROS has been initialized
  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(
      impl_->model_nh_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
        "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Hold joints if no control mode is active?
  bool hold_joints = true;  // default
  if (sdf->HasElement("hold_joints")) {
    hold_joints =
      sdf->GetElement("hold_joints")->Get<bool>();
  }

  // Get controller manager node name
  std::string controllerManagerNodeName{"controller_manager"};

  if (sdf->HasElement("controller_manager_name")) {
    controllerManagerNodeName = sdf->GetElement("controller_manager_name")->Get<std::string>();
  }

  // There's currently no direct way to set parameters to the plugin's node
  // So we have to parse the plugin file manually and set it to the node's context.
  auto rcl_context = impl_->model_nh_->get_node_base_interface()->get_context()->get_rcl_context();
  std::vector<std::string> arguments = {"--ros-args"};

  if (sdf->HasElement("parameters")) {
    sdf::ElementPtr argument_sdf = sdf->GetElement("parameters");
    while (argument_sdf) {
      std::string argument = argument_sdf->Get<std::string>();
      RCLCPP_INFO(impl_->model_nh_->get_logger(), "Loading parameter files %s", argument.c_str());
      arguments.push_back(RCL_PARAM_FILE_FLAG);
      arguments.push_back(argument);
      argument_sdf = argument_sdf->GetNextElement("parameters");
    }
  } else {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "No parameter file provided. Configuration might be wrong");
  }

  if (sdf->HasElement("ros")) {
    sdf = sdf->GetElement("ros");
    // Get list of remapping rules from SDF
    if (sdf->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdf->GetElement("remapping");

      while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("remapping");
      }
    }
  }

  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  rcl_arguments_t rcl_args = rcl_get_zero_initialized_arguments();
  rcl_ret_t rcl_ret = rcl_parse_arguments(
    static_cast<int>(argv.size()),
    argv.data(), rcl_get_default_allocator(), &rcl_args);
  rcl_context->global_arguments = rcl_args;
  if (rcl_ret != RCL_RET_OK) {
    RCLCPP_ERROR(impl_->model_nh_->get_logger(), "parser error %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  if (rcl_arguments_get_param_files_count(&rcl_args) < 1) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "failed to parse input yaml file(s)");
    return;
  }

  // Get the Gazebo simulation period
  rclcpp::Duration gazebo_period(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        impl_->parent_model_->GetWorld()->Physics()->GetMaxStepSize())));

  rclcpp::Node::SharedPtr node_ros2 = std::dynamic_pointer_cast<rclcpp::Node>(
    impl_->model_nh_);
  try {
    node_ros2->declare_parameter("hold_joints", rclcpp::ParameterValue(hold_joints));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "Parameter 'hold_joints' has already been declared, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException & e) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "Parameter 'hold_joints' has invalid name, %s", e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "Parameter 'hold_joints' value is invalid, %s", e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "Parameter 'hold_joints' value has wrong type, %s",
      e.what());
  }

  impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<gazebo_ros2_control::GazeboResourceManager>(
    node_ros2,
    impl_->parent_model_, sdf);

  // Create the controller manager
  RCLCPP_INFO(impl_->model_nh_->get_logger(), "Loading controller_manager");
  rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
  // Force setting of use_sime_time parameter
  arguments.push_back("--param");
  arguments.push_back("use_sim_time:=True");
  options.arguments(arguments);
  impl_->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      impl_->executor_,
      controllerManagerNodeName,
      impl_->model_nh_->get_namespace(), options));
  impl_->executor_->add_node(impl_->controller_manager_);

  auto cm_update_rate = impl_->controller_manager_->get_update_rate();
  impl_->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));
  // Check the period against the simulation period
  if (impl_->control_period_ < gazebo_period) {
    RCLCPP_ERROR_STREAM(
      impl_->model_nh_->get_logger(),
      "Desired controller update period (" << impl_->control_period_.seconds() <<
        " s) is faster than the gazebo simulation period (" <<
        gazebo_period.seconds() << " s).");
  } else if (impl_->control_period_ > gazebo_period) {
    RCLCPP_WARN_STREAM(
      impl_->model_nh_->get_logger(),
      " Desired controller update period (" << impl_->control_period_.seconds() <<
        " s) is slower than the gazebo simulation period (" <<
        gazebo_period.seconds() << " s).");
  }

  impl_->stop_ = false;
  auto spin = [this]()
    {
      while (rclcpp::ok() && !impl_->stop_) {
        impl_->executor_->spin_once();
      }
    };
  impl_->thread_executor_spin_ = std::thread(spin);

  // Listen to the update event. This event is broadcast every simulation iteration.
  impl_->update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(
      &GazeboRosControlPrivate::Update,
      impl_.get()));

  // Wait for CM to receive robot description from the topic and then initialize Resource Manager
  while (!impl_->controller_manager_->is_resource_manager_initialized()) {
    RCLCPP_WARN(
      impl_->model_nh_->get_logger(),
      "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::microseconds(2000000));
  }

  RCLCPP_INFO(impl_->model_nh_->get_logger(), "Loaded gazebo_ros2_control.");
}

// Called by the world update start event
void GazeboRosControlPrivate::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  // use same time as for read and update call - this is how it is done is ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);
}

// Called on world reset
void GazeboRosControlPrivate::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosControlPlugin)
}  // namespace gazebo_ros2_control
