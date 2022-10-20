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
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "interfaces.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("moveit_constraints_demo");
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


  RCLCPP_INFO(logger, "step2: move end effector without constraints");

  moveit::planning_interface::MoveGroupInterface whole_body_group(move_group_node, "whole_body_weighted");
  whole_body_group.setMaxVelocityScalingFactor(0.5);
  whole_body_group.setMaxAccelerationScalingFactor(0.5);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = whole_body_group.getPlanningFrame();
  collision_object.id = "table";

  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[table_primitive.BOX_X] = 0.4;
  table_primitive.dimensions[table_primitive.BOX_Y] = 0.8;
  table_primitive.dimensions[table_primitive.BOX_Z] = 0.01;

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.5;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.8 + table_primitive.dimensions[table_primitive.BOX_Z] / 2.0;

  collision_object.primitives.push_back(table_primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {collision_object};

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.z = 1.0;
  target_pose.orientation.x = 0.707;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.707;
  target_pose.orientation.w = 0.0;

  whole_body_group.setPoseTarget(target_pose);
  whole_body_group.move();

  rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));


  RCLCPP_INFO(logger, "step3: move end effector with constraints");

  planning_scene_interface.removeCollisionObjects({collision_object.id});
  if (!interfaces.MoveToNeutral()) {
    RCLCPP_ERROR(logger, "MoveToNeutral failure");
    return EXIT_FAILURE;
  }

  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.link_name = whole_body_group.getEndEffectorLink();
  orientation_constraint.header.frame_id = whole_body_group.getPlanningFrame();
  orientation_constraint.orientation = target_pose.orientation;
  orientation_constraint.absolute_x_axis_tolerance = 4.0;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = 0.1;
  orientation_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints = {orientation_constraint};
  whole_body_group.setPathConstraints(constraints);

  planning_scene_interface.addCollisionObjects(collision_objects);
  whole_body_group.move();

  planning_scene_interface.removeCollisionObjects({collision_object.id});

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
