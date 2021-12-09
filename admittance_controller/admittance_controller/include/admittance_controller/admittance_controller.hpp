// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Denis Stogl, Andy Zelenak

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule.hpp"
#include "admittance_controller/tf2_eigen.hpp"
#include "admittance_controller/tf2_geometry_msgs.hpp"
#include "admittance_controller/visibility_control.h"
#include "admittance_controller_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"
// TODO(destogl): this is only temporary to work with servo. It should be either trajectory_msgs/msg/JointTrajectoryPoint or std_msgs/msg/Float64MultiArray
#include "trajectory_msgs/msg/joint_trajectory.hpp"

//#include "joint_trajectory_controller/trajectory_execution_impl.hpp"
#include "joint_trajectory_controller/tolerances.hpp"

namespace admittance_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface,
                             public joint_trajectory_controller::TrajectoryExecutionImpl
{
public:
  ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<double> joint_deltas_;
  std::string ft_sensor_name_;
  bool use_joint_commands_as_input_;
  std::string joint_limiter_type_;

  // Parameters for some special cases, e.g. hydraulics powered robots
  /// Run he controller in open-loop, i.e., read hardware states only when starting controller.
  /// This is useful when robot is not exactly following the commanded trajectory.
  bool hardware_state_has_offset_;
  bool open_loop_control_ = false;
  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
  trajectory_msgs::msg::JointTrajectoryPoint last_state_reference_;

  // joint limiter
  using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointLimits>;
  std::shared_ptr<pluginlib::ClassLoader<JointLimiter>> joint_limiter_loader_;
  std::unique_ptr<JointLimiter> joint_limiter_;

  /// Allow integration in goal trajectories to accept goals without position or velocity specified
  bool allow_integration_in_goal_trajectories_ = false;

  // Internal variables
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // Admittance rule and dependent variables;
  std::unique_ptr<admittance_controller::AdmittanceRule> admittance_;
  rclcpp::Time previous_time_;

  // Callback for updating dynamic parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

  // Command subscribers and Controller State publisher
  using ControllerCommandWrenchMsg = geometry_msgs::msg::WrenchStamped;
  using ControllerCommandPoseMsg = geometry_msgs::msg::PoseStamped;
  using ControllerCommandJointMsg = trajectory_msgs::msg::JointTrajectory;

  rclcpp::Subscription<ControllerCommandWrenchMsg>::SharedPtr
  input_wrench_command_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerCommandPoseMsg>::SharedPtr
  input_pose_command_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerCommandJointMsg>::SharedPtr input_joint_command_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandWrenchMsg>>
  input_wrench_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandPoseMsg>>
  input_pose_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandJointMsg>>
  input_joint_command_;

  using ControllerStateMsg = admittance_controller_msgs::msg::AdmittanceControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  bool allow_partial_joints_goal_ = false;

  joint_trajectory_controller::SegmentTolerances default_tolerances_;

  // Internal access to sorted interfaces

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;

  void read_state_from_hardware(trajectory_msgs::msg::JointTrajectoryPoint & state);

  /// Use values on command interfaces as states when robot should be controller in open-loop.
  /**
   * If velocities and positions are both available from the joint command interface, set
   * output_state equal to them.
   * If velocities or positions are unknown, output_state is unchanged and the function returns false.
   */
  bool read_state_from_command_interfaces(trajectory_msgs::msg::JointTrajectoryPoint & state);
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
