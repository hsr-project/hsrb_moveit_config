#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OdomJointStatePublisher(Node):

    def __init__(self):
        """Use that when the ROS2 migration of Topic_tools is completed, a node that issues ODOM_X/Y/T in JointState."""
        super().__init__('odom_joint_state_publisher')
        self._publisher = self.create_publisher(JointState, 'odom_joint_states', 1)
        self._subscription = self.create_subscription(
            JointTrajectoryControllerState, 'omni_base_controller/state', self._callback, 1)

    def _callback(self, msg):
        output = JointState()
        output.header = msg.header
        output.name = msg.joint_names
        output.position = msg.actual.positions
        output.velocity = msg.actual.velocities
        self._publisher.publish(output)


def main():
    rclpy.init()
    relay = OdomJointStatePublisher()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
