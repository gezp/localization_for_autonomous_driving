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
    pkg_lidar_mapping = get_package_share_directory("lidar_mapping")
    rviz2_config = os.path.join(pkg_lidar_mapping, "launch", "mapping.rviz")
    front_end_config = os.path.join(pkg_lidar_mapping, "config", "front_end.yaml")
    back_end_config = os.path.join(pkg_lidar_mapping, "config", "back_end.yaml")
    loop_closure_config = os.path.join(pkg_lidar_mapping, "config", "loop_closure.yaml")
    data_dir = os.path.join(os.environ["HOME"], "localization_data")
    bag_path = os.path.join(data_dir, "kitti_lidar_only_2011_10_03_drive_0027_synced")
    rosbag_node = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--read-ahead-queue-size", "10000"],
        output="screen",
    )
    kitti_preprocess_node = Node(
        name="kitti_preprocess_node",
        package="localization_common",
        executable="kitti_preprocess_node",
        output="screen",
    )
    mapping_front_end_node = Node(
        name="mapping_front_end_node",
        package="lidar_mapping",
        executable="mapping_front_end_node",
        parameters=[{"front_end_config": front_end_config}],
        output="screen",
    )
    mapping_back_end_node = Node(
        name="mapping_back_end_node",
        package="lidar_mapping",
        executable="mapping_back_end_node",
        parameters=[{"back_end_config": back_end_config, "data_path": data_dir}],
        output="screen",
    )
    mapping_loop_closure_node = Node(
        name="mapping_loop_closure_node",
        package="lidar_mapping",
        executable="mapping_loop_closure_node",
        parameters=[
            {"loop_closure_config": loop_closure_config, "data_path": data_dir}
        ],
        output="screen",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(rosbag_node)
    ld.add_action(kitti_preprocess_node)
    ld.add_action(mapping_front_end_node)
    ld.add_action(mapping_back_end_node)
    ld.add_action(mapping_loop_closure_node)
    ld.add_action(rviz2)
    return ld
