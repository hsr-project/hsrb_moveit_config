#!/usr/bin/env python
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
import os
import sys

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


try:
    from utils import (
        get_full_path,
        load_file,
        load_yaml,
    )
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    from utils import (
        get_full_path,
        load_file,
        load_yaml,
    )


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('robot_description',
                              default_value='robot_description',
                              description='Robot description for moveit.'))
    declared_arguments.append(
        DeclareLaunchArgument('robot_name', default_value='hsrb', choices=['hsrb', 'hsrc'],
                              description='Robot name for kinematics plugin.'))
    declared_arguments.append(
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'],
                              description='Launch rviz with moveit.'))
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Launch with simulator.'))
    return declared_arguments


def generate_launch_description():
    robot_description = {'robot_description': LaunchConfiguration('robot_description')}
    robot_description_semantic = {'robot_description_semantic': load_file('config/hsrb.srdf')}
    robot_description_planning = {'robot_description_planning': load_yaml('config/joint_limits.yaml')}
    kinematics_yaml = load_yaml('config/kinematics.yaml')
    sensors_yaml = load_yaml('config/sensors_xtion.yaml')
    robot_name = {'robot_name': LaunchConfiguration('robot_name')}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join(['default_planner_request_adapters/AddTimeOptimalParameterization',
                                          'default_planner_request_adapters/FixWorkspaceBounds',
                                          'default_planner_request_adapters/FixStartStateBounds',
                                          'default_planner_request_adapters/FixStartStateCollision',
                                          'default_planner_request_adapters/FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}}
    ompl_planning_pipeline_config['move_group'].update(load_yaml('config/ompl_planning.yaml'))

    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': load_yaml('config/hsrb_controllers.yaml')}

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           output='screen',
                           parameters=[robot_description,
                                       robot_description_semantic,
                                       robot_description_planning,
                                       kinematics_yaml,
                                       sensors_yaml,
                                       robot_name,
                                       ompl_planning_pipeline_config,
                                       trajectory_execution,
                                       moveit_controllers,
                                       planning_scene_monitor_parameters,
                                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                           remappings=[('joint_states', 'whole_body_moveit/joint_states')])

    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'world'])

    odom_joint_states_publisher = Node(package='hsrb_moveit_config',
                                       executable='odom_joint_states_publisher.py',
                                       name='odom_joint_states_publisher')

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 namespace='whole_body_moveit',
                                 arguments=['/tmp/robot_description.urdf'],
                                 parameters=[{'source_list': ['/joint_states', '/odom_joint_states']}],
                                 remappings=[('robot_description', '/robot_description')])

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml,
                                 {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                     arguments=['-d', get_full_path('config/moveit.rviz')],
                     condition=IfCondition(LaunchConfiguration('use_rviz')))

    nodes = [move_group_node, static_tf, odom_joint_states_publisher, joint_state_publisher, rviz_node]

    return LaunchDescription(declare_arguments() + nodes)
