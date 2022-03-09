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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        'description',
        default_value='false',
        description='Launch turtlebot4 description'
    ),
    DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='Turtlebot4 Model'
    )
]


def generate_launch_description():

    pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'rviz', 'robot.rviz'])
    description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    # Delay launch of robot description to allow Rviz2 to load first.
    # Prevents visual bugs in the model.
    robot_description = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([description_launch]),
                launch_arguments=[('model', LaunchConfiguration('model'))],
                condition=IfCondition(LaunchConfiguration('description'))
            )]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description)
    ld.add_action(rviz2)
    return ld
