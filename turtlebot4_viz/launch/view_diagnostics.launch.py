# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    robot_monitor = Node(
        package='rqt_robot_monitor',
        executable='rqt_robot_monitor',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(robot_monitor)
    return ld
