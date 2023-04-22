# localization_for_autonomous_driving

[![Build and Test](https://github.com/gezp/localization_for_autonomous_driving/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/gezp/localization_for_autonomous_driving/actions/workflows/ci.yml)

该项目基于ROS2平台实现了自动驾驶中的简单定位功能，主要面向初学者学习入门，做这个项目的出发点源于本人在完成深蓝学院的[多传感器融合定位](https://www.shenlanxueyuan.com/course/558)课程后，有了对原始项目进行重构的想法，一方面可以回顾总结经验，提高对定位工程的理解，另一方面是提供一个用于定位学习的ROS2包，方便后续的学习者进行学习与扩展，避免环境搭建难的问题。

本项目的主要框架以及核心代码参考了任乾老师的[从零开始做自动驾驶定位](https://github.com/Little-Potato-1990/localization_in_auto_driving)项目，以及Yao Ge大佬提供的课程项目代码[Sensor-Fusion-for-Localization-Courseware](https://github.com/AlexGeControl/Sensor-Fusion-for-Localization-Courseware)。

## 快速开始

环境要求

* ROS版本: `Humble`

下载kitti数据集rosbag

* [腾讯微云](https://share.weiyun.com/Ck2PB1wp)

```bash
mkdir ~/localization_data
# 将数据拷贝到~/localization_data目录下，然后进行解压
unzip kitti_lidar_only_2011_10_03_drive_0027_synced.zip 
```

下载源码及安装依赖

```bash
# 使用git下载代码到ROS2工作空间的src目录
# mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/gezp/localization_for_autonomous_driving.git
# 进入ROS2工作空间, 安装依赖
# cd  ~/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
```

编译

```bash
colcon build --symlink-install
```

运行kitti数据集点云可视化demo

```bash
# 需要先source ros环境
ros2 launch localization_common hello_kitti.launch.py
```

## 项目计划

lidar定位

- [x] 实现lidar里程计功能
- [x] 实现基于lidar的建图功能
- [x] 实现基于lidar的定位功能

lidar + imu 多传感器融合定位

- [x] 实现基于eskf的定位功能
- [ ] 实现松耦合的LIO建图功能
- [ ] 实现基于LIO优化的定位功能

本项目将遵循ROS2的项目规范，以及代码风格，因此在实现过程中将会耗费较多的时间上在代码质量上，目前预计3个月时间完成所有内容。

