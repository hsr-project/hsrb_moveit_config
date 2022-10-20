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

  auto move_group_node = rclcpp::Node::make_shared("moveit_gripper_demo");
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


  RCLCPP_INFO(logger, "step2: open");

  moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, "gripper");
  gripper_group.setJointValueTarget({"hand_motor_joint"}, {1.0});
  gripper_group.setMaxAccelerationScalingFactor(1.0);
  gripper_group.setMaxVelocityScalingFactor(1.0);
  gripper_group.move();

  rclcpp::sleep_for(std::chrono::nanoseconds(2000000000));


  RCLCPP_INFO(logger, "step3: close");

  gripper_group.setJointValueTarget({"hand_motor_joint"}, {0.0});
  gripper_group.move();

  rclcpp::sleep_for(std::chrono::nanoseconds(2000000000));


  RCLCPP_INFO(logger, "step4: open");

  gripper_group.setJointValueTarget({"hand_motor_joint"}, {1.0});
  gripper_group.move();

  rclcpp::sleep_for(std::chrono::nanoseconds(2000000000));


  RCLCPP_INFO(logger, "step5: grasp");

  auto msg = moveit_msgs::msg::RobotTrajectory();
  msg.joint_trajectory.joint_names = {"hand_motor_joint"};
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = {1.0};
  point.effort = {-0.01};
  point.time_from_start = rclcpp::Duration(0, 0);
  msg.joint_trajectory.points.push_back(point);

  point.positions = {0.0};
  point.effort = {-0.01};
  point.time_from_start = rclcpp::Duration(1, 0);
  msg.joint_trajectory.points.push_back(point);

  gripper_group.execute(msg);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
