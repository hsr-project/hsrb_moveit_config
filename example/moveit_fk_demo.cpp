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
#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("moveit_fk_demo");
  auto logger = move_group_node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();


  RCLCPP_INFO(logger, "step1: move_to_neutral");

  moveit::planning_interface::MoveGroupInterface arm_group(move_group_node, "arm");
  arm_group.setMaxVelocityScalingFactor(0.5);
  arm_group.setMaxAccelerationScalingFactor(0.5);
  if (!arm_group.setNamedTarget("neutral")) {
    RCLCPP_ERROR(logger, "'neutral' is not in 'arm' group");
    return EXIT_FAILURE;
  }
  arm_group.move();

  moveit::planning_interface::MoveGroupInterface head_group(move_group_node, "head");
  if (!head_group.setNamedTarget("neutral")) {
    RCLCPP_ERROR(logger, "'neutral' is not in 'head' group");
    return EXIT_FAILURE;
  }
  head_group.move();


  RCLCPP_INFO(logger, "step2: move_to_go");

  if (!arm_group.setNamedTarget("go")) {
    RCLCPP_ERROR(logger, "'go' is not in 'arm' group");
    return EXIT_FAILURE;
  }
  arm_group.move();


  RCLCPP_INFO(logger, "step3: get joint names");

  for (const auto joint_name : arm_group.getActiveJoints()) {
    RCLCPP_INFO_STREAM(logger, "- " << joint_name);
  }


  RCLCPP_INFO(logger, "step4: set arm_lift_joint 0.2");

  arm_group.setJointValueTarget({"arm_lift_joint"}, {0.2});
  arm_group.move();


  RCLCPP_INFO(logger, "step5: move_whole_body");

  moveit::planning_interface::MoveGroupInterface whole_body_group(move_group_node, "whole_body");
  whole_body_group.setMaxVelocityScalingFactor(0.5);
  whole_body_group.setMaxAccelerationScalingFactor(0.5);
  whole_body_group.setJointValueTarget({"odom_x", "odom_y", "odom_t",
                                        "arm_lift_joint", "arm_flex_joint", "wrist_flex_joint"},
                                       {0.2, -0.2, 0.5, 0.1, -0.5, -1.07});
  whole_body_group.move();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
