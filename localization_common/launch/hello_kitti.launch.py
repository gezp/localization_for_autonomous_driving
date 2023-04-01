# Copyright 2023 Gezp (https://github.com/gezp).
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization_common = get_package_share_directory("localization_common")
    rviz2_config = os.path.join(pkg_localization_common, "launch", "hello_kitti.rviz")
    data_dir = os.path.join(os.environ["HOME"], "localization_data")
    bag_path = os.path.join(data_dir, "kitti_lidar_only_2011_10_03_drive_0027_synced")
    rosbag_node = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--read-ahead-queue-size", "10000"],
        output="screen",
    )
    hello_kitti_node = Node(
        name="hello_kitti_node",
        package="localization_common",
        executable="hello_kitti_node",
        output="screen",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config],
        output="screen",
    )
    # avoid rviz warning that frame 'map' and 'velo_link' does not exist.
    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "faker1"],
        output="screen",
    )
    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "velo_link", "--child-frame-id", "faker2"],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(rosbag_node)
    ld.add_action(hello_kitti_node)
    ld.add_action(rviz2)
    ld.add_action(tf_node)
    ld.add_action(tf2_node)
    return ld
