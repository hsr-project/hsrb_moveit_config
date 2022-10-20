/*
Copyright (c) 2022 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include "interfaces.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("moveit_ik_demo");
  auto logger = move_group_node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();


  RCLCPP_INFO(logger, "step1: move_to_neutral");

  auto interfaces = hsrb_moveit_config::Interfaces(move_group_node);
  if (!interfaces.MoveToNeutral()) {
    RCLCPP_ERROR(logger, "MoveToNeutral failure");
    return EXIT_FAILURE;
  }


  RCLCPP_INFO(logger, "step2: move hand forward");

  moveit::planning_interface::MoveGroupInterface whole_body_group(move_group_node, "whole_body");
  whole_body_group.setMaxVelocityScalingFactor(0.5);
  whole_body_group.setMaxAccelerationScalingFactor(0.5);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 1.0;
  target_pose.position.z = 0.7;
  target_pose.orientation.x = 0.707;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.707;
  target_pose.orientation.w = 0.0;
  whole_body_group.setPoseTarget(target_pose);
  whole_body_group.move();

  if (!interfaces.MoveToNeutral()) {
    RCLCPP_ERROR(logger, "MoveToNeutral failure");
    return EXIT_FAILURE;
  }


  RCLCPP_INFO(logger, "step3: move hand forward (weighted)");

  moveit::planning_interface::MoveGroupInterface whole_body_weighted_group(move_group_node, "whole_body_weighted");
  whole_body_weighted_group.setMaxVelocityScalingFactor(0.5);
  whole_body_weighted_group.setMaxAccelerationScalingFactor(0.5);

  whole_body_weighted_group.setPoseTarget(target_pose);
  whole_body_weighted_group.move();

  if (!interfaces.MoveToNeutral()) {
    RCLCPP_ERROR(logger, "MoveToNeutral failure");
    return EXIT_FAILURE;
  }


  RCLCPP_INFO(logger, "step4: move hand forward (light)");

  moveit::planning_interface::MoveGroupInterface whole_body_light_group(move_group_node, "whole_body_light");
  whole_body_light_group.setMaxVelocityScalingFactor(0.5);
  whole_body_light_group.setMaxAccelerationScalingFactor(0.5);

  whole_body_light_group.setPoseTarget(target_pose);
  whole_body_light_group.move();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
