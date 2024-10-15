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
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def parse(description_package, description_file):
    xacro_file = os.path.join(get_package_share_directory(description_package), 'robots', description_file)
    robot_description_content = parse_xacro(xacro_file)

    root = add_joints_and_links(robot_description_content)

    tree = ET.ElementTree(root)
    tree.write('/tmp/robot_description.urdf')

    return ET.tostring(root, encoding='unicode', method='xml')


def parse_xacro(xacro_file):
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    return doc.toxml()


##
# Software License Agreement (BSD License)
#
#  Copyright (c) 2020, MID Academic Promotions, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the MID Academic Promotions nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
##


def add_joints_and_links(robot_description_content):
    root = ET.fromstring(robot_description_content)
    root.append(link('odom'))
    root.append(link('odom_xt_link'))
    root.append(link('odom_yx_link'))
    root.append(odom_t_joint())
    root.append(odom_x_joint())
    root.append(odom_y_joint())
    return root


def odom_t_joint():
    j = ET.Element('joint', {'name': 'odom_t', 'type': 'continuous'})
    j.append(ET.Element('axis', {'xyz': '0 0 1'}))
    j.append(ET.Element('parent', {'link': 'odom_xt_link'}))
    j.append(ET.Element('child', {'link': 'base_footprint'}))
    j.append(ET.Element('limit', {'effort': '0.1', 'velocity': '1'}))
    return (j)


def odom_x_joint():
    j = ET.Element('joint', {'name': 'odom_x', 'type': 'prismatic'})
    j.append(ET.Element('axis', {'xyz': '1 0 0'}))
    j.append(ET.Element('parent', {'link': 'odom_yx_link'}))
    j.append(ET.Element('child', {'link': 'odom_xt_link'}))
    j.append(ET.Element('limit', {'lower': '-10', 'upper': '10', 'effort': '0.1', 'velocity': '0.2'}))
    return (j)


def odom_y_joint():
    j = ET.Element('joint', {'name': 'odom_y', 'type': 'prismatic'})
    j.append(ET.Element('axis', {'xyz': '0 1 0'}))
    j.append(ET.Element('parent', {'link': 'odom'}))
    j.append(ET.Element('child', {'link': 'odom_yx_link'}))
    j.append(ET.Element('limit', {'lower': '-10', 'upper': '10', 'effort': '0.1', 'velocity': '0.2'}))
    return (j)


def link(name):
    link = ET.Element('link', {'name': name})
    i = ET.Element('inertial')  # inertial element is required to pass urdf to sdf conversion in gazebo
    mass = 0.1
    inertia = 2.0 * mass * 0.01 * 0.01 / 5.0  # assume as sphere
    i.append(ET.Element('mass', {'value': str(mass)}))
    i.append(ET.Element('inertia', {'ixx': str(inertia), 'ixy': '0',
             'ixz': '0', 'iyy': str(inertia), 'iyz': '0', 'izz': str(inertia)}))
    link.append(i)
    return (link)
