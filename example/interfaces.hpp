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
#ifndef HSRB_MOVEIT_CONFIG_EXAMPLE_INTERFACES_HPP_
#define HSRB_MOVEIT_CONFIG_EXAMPLE_INTERFACES_HPP_

#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace hsrb_moveit_config {

struct Interfaces {
  moveit::planning_interface::MoveGroupInterface arm_group;
  moveit::planning_interface::MoveGroupInterface head_group;
  moveit::planning_interface::MoveGroupInterface base_group;

  explicit Interfaces(const rclcpp::Node::SharedPtr& node, double scaling_factor = 0.5)
      : arm_group(node, "arm"),
        head_group(node, "head"),
        base_group(node, "base") {
    arm_group.setMaxVelocityScalingFactor(scaling_factor);
    arm_group.setMaxAccelerationScalingFactor(scaling_factor);
    head_group.setMaxVelocityScalingFactor(scaling_factor);
    head_group.setMaxAccelerationScalingFactor(scaling_factor);
    base_group.setMaxVelocityScalingFactor(scaling_factor);
    base_group.setMaxAccelerationScalingFactor(scaling_factor);
  }

  bool MoveToNeutral() {
    if (!arm_group.setNamedTarget("neutral")) {
      return false;
    }
    arm_group.move();

    if (!head_group.setNamedTarget("neutral")) {
      return false;
    }
    head_group.move();

    if (!base_group.setNamedTarget("neutral")) {
      return false;
    }
    base_group.move();

    return true;
  }
};

}  // namespace hsrb_moveit_config
#endif  // HSRB_MOVEIT_CONFIG_EXAMPLE_INTERFACES_HPP_
