// Copyright 2020 PAL Robotics S.L.
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

#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "position_controllers/joint_group_position_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace position_controllers
{
using CallbackReturn = JointGroupPositionController::CallbackReturn;

JointGroupPositionController::JointGroupPositionController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "joint position controller";
  interface_name_ = hardware_interface::HW_IF_POSITION;
}

controller_interface::return_type
JointGroupPositionController::init(const std::string & controller_name)
{
  auto ret = ForwardCommandController::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  return controller_interface::return_type::OK;
}
}  // namespace position_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  position_controllers::JointGroupPositionController, controller_interface::ControllerInterface)
