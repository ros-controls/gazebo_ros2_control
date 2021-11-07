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
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "gazebo_ros/node.hpp"

#include "gazebo_ros2_control/gazebo_ros2_control_plugin.hpp"
#include "gazebo_ros2_control/gazebo_system.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "urdf/model.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace gazebo_ros2_control
{

class GazeboRosControlPrivate
{
public:
  GazeboRosControlPrivate() = default;

  virtual ~GazeboRosControlPrivate() = default;

  // Called by the world update start event
  void Update();

  // Called on world reset
  virtual void Reset();

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;

  // Node Handles
  gazebo_ros::Node::SharedPtr model_nh_;

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<
      gazebo_ros2_control::GazeboSystemInterface>> robot_hw_sim_loader_;

  // String with the robot description
  std::string robot_description_;

  // String with the name of the node that contains the robot_description
  std::string robot_description_node_;

  // Name of the file with the controllers configuration
  std::string param_file_;

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
    impl_->model_nh_->get_logger(), "Starting gazebo_ros2_control plugin in ros 2 node: %s",
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

  // Get robot_description ROS param name
  if (sdf->HasElement("robot_param")) {
    impl_->robot_description_ = sdf->GetElement("robot_param")->Get<std::string>();
  } else {
    impl_->robot_description_ = "robot_description";  // default
  }

  // Get robot_description ROS param name
  if (sdf->HasElement("robot_param_node")) {
    impl_->robot_description_node_ =
      sdf->GetElement("robot_param_node")->Get<std::string>();
  } else {
    impl_->robot_description_node_ = "robot_state_publisher";  // default
  }

  if (sdf->HasElement("parameters")) {
    impl_->param_file_ = sdf->GetElement("parameters")->Get<std::string>();
    RCLCPP_INFO(
      impl_->model_nh_->get_logger(), "Loading parameter file %s\n", impl_->param_file_.c_str());
  } else {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "No parameter file provided. Configuration might be wrong");
  }

  // There's currently no direct way to set parameters to the plugin's node
  // So we have to parse the plugin file manually and set it to the node's context.
  auto rcl_context = impl_->model_nh_->get_node_base_interface()->get_context()->get_rcl_context();
  std::vector<std::string> arguments = {"--ros-args", "--params-file", impl_->param_file_.c_str()};
  if (sdf->HasElement("ros")) {
    sdf = sdf->GetElement("ros");

    // Set namespace if tag is present
    if (sdf->HasElement("namespace")) {
      std::string ns = sdf->GetElement("namespace")->Get<std::string>();
      // prevent exception: namespace must be absolute, it must lead with a '/'
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }
      std::string ns_arg = std::string("__ns:=") + ns;
      arguments.push_back(RCL_REMAP_FLAG);
      arguments.push_back(ns_arg);
    }

    // Get list of remapping rules from SDF
    if (sdf->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdf->GetElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
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
      impl_->model_nh_->get_logger(), "failed to parse yaml file: '%s'\n",
      impl_->param_file_.c_str());
    return;
  }

  // Get the Gazebo simulation period
  rclcpp::Duration gazebo_period(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        impl_->parent_model_->GetWorld()->Physics()->GetMaxStepSize())));

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware;
  try {
    urdf_string = impl_->getURDF(impl_->robot_description_);
    control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_STREAM(
      impl_->model_nh_->get_logger(),
      "Error parsing URDF in gazebo_ros2_control plugin, plugin not active : " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<hardware_interface::ResourceManager>();

  try {
    impl_->robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface>(
        "gazebo_ros2_control",
        "gazebo_ros2_control::GazeboSystemInterface"));
  } catch (pluginlib::LibraryLoadException & ex) {
    RCLCPP_ERROR(
      impl_->model_nh_->get_logger(), "Failed to create robot simulation interface loader: %s ",
      ex.what());
  }

  for (unsigned int i = 0; i < control_hardware.size(); i++) {
    std::string robot_hw_sim_type_str_ = control_hardware[i].hardware_class_type;
    auto gazeboSystem = std::unique_ptr<gazebo_ros2_control::GazeboSystemInterface>(
      impl_->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));

    rclcpp::Node::SharedPtr node_ros2 = std::dynamic_pointer_cast<rclcpp::Node>(impl_->model_nh_);
    if (!gazeboSystem->initSim(
        node_ros2,
        impl_->parent_model_,
        control_hardware[i],
        sdf))
    {
      RCLCPP_FATAL(
        impl_->model_nh_->get_logger(), "Could not initialize robot simulation interface");
      return;
    }

    resource_manager_->import_component(std::move(gazeboSystem), control_hardware[i]);
  }

  impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Create the controller manager
  RCLCPP_INFO(impl_->model_nh_->get_logger(), "Loading controller_manager");
  impl_->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      impl_->executor_,
      "controller_manager"));
  impl_->executor_->add_node(impl_->controller_manager_);

  if (!impl_->controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
      impl_->model_nh_->get_logger(), "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto cm_update_rate = impl_->controller_manager_->get_parameter("update_rate").as_int();
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
    controller_manager_->read();
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  controller_manager_->write();
}

// Called on world reset
void GazeboRosControlPrivate::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
}

// Get the URDF XML from the parameter server
std::string GazeboRosControlPrivate::getURDF(std::string param_name) const
{
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    model_nh_, robot_description_node_);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        model_nh_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
        robot_description_node_.c_str());
      return 0;
    }
    RCLCPP_ERROR(
      model_nh_->get_logger(), "%s service not available, waiting again...",
      robot_description_node_.c_str());
  }

  RCLCPP_INFO(
    model_nh_->get_logger(), "connected to service!! %s", robot_description_node_.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    std::string search_param_name;
    RCLCPP_DEBUG(model_nh_->get_logger(), "param_name %s", param_name.c_str());

    try {
      auto f = parameters_client->get_parameters({param_name});
      f.wait();
      std::vector<rclcpp::Parameter> values = f.get();
      urdf_string = values[0].as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(model_nh_->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(
        model_nh_->get_logger(), "gazebo_ros2_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());
    }
    usleep(100000);
  }
  RCLCPP_INFO(
    model_nh_->get_logger(), "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosControlPlugin)
}  // namespace gazebo_ros2_control
