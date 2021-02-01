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

#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"

#include "gazebo/common/common.hh"
#include "gazebo/physics/Model.hh"

namespace gazebo_ros2_control
{
class GazeboRosControlPrivate;

class GazeboRosControlPlugin : public gazebo::ModelPlugin
{
public:
  GazeboRosControlPlugin();

  ~GazeboRosControlPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosControlPrivate> impl_;
};
}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
