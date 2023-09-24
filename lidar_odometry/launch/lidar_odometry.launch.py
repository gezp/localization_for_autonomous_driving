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
    pkg_lidar_odometry = get_package_share_directory("lidar_odometry")
    rviz2_config = os.path.join(pkg_lidar_odometry, "launch", "lidar_odometry.rviz")
    data_dir = os.path.join(os.environ["HOME"], "localization_data")
    lidar_odometry_config = os.path.join(
        pkg_lidar_odometry, "config", "lidar_odometry.yaml"
    )
    bag_path = os.path.join(data_dir, "kitti_lidar_only_2011_10_03_drive_0027_synced")
    rosbag_node = ExecuteProcess(
        name="rosbag",
        cmd=["ros2 bag play", bag_path, "-d 3", "--read-ahead-queue-size 1000"],
        shell=True,
        output="screen",
    )
    kitti_preprocess_node = Node(
        name="kitti_preprocess_node",
        package="localization_common",
        executable="kitti_preprocess_node",
        output="screen",
    )
    lidar_odometry_node = Node(
        name="lidar_odometry_node",
        package="lidar_odometry",
        executable="lidar_odometry_node",
        parameters=[
            {
                "lidar_odometry_config": lidar_odometry_config,
                "publish_tf": True,
                "use_initial_pose_from_topic": True,
                "base_frame_id": "base_link",
                "lidar_frame_id": "velo_link",
            }
        ],
        remappings=[("reference_odom", "/synced_gnss/pose")],
        output="screen",
    )
    simple_evaluator_node = Node(
        name="simple_evaluator_node",
        package="localization_common",
        executable="simple_evaluator_node",
        parameters=[
            {
                "data_path": data_dir,
                "odom_names": ["ground_truth", "lidar_odom"],
                "odom_topics": ["synced_gnss/pose", "lidar_odom"],
            }
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
    ld.add_action(lidar_odometry_node)
    ld.add_action(simple_evaluator_node)
    ld.add_action(rviz2)
    return ld
