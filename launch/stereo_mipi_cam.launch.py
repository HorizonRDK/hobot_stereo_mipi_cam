# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'io_method',
            default_value='ros',
            description='ros/shared_mem'),
        DeclareLaunchArgument(
            'image_width',
            default_value='1920',
            description='camera out image width'),
        DeclareLaunchArgument(
            'image_height',
            default_value='1080',
            description='camera out image height'),
        DeclareLaunchArgument(
            'data_sampling_rate',
            default_value='-1',
            description='camera sampling rate'),

        Node(
            package='stereo_mipi_cam',
            executable='stereo_mipi_cam',
            output='screen',
            parameters=[
                {"io_method": LaunchConfiguration('io_method')},
                {"image_width": LaunchConfiguration('image_width')},
                {"image_height": LaunchConfiguration('image_height')},
                {"data_sampling_rate": LaunchConfiguration('data_sampling_rate')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
