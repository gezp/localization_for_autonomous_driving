# localization_for_autonomous_driving

[![Build and Test](https://github.com/gezp/localization_for_autonomous_driving/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/gezp/localization_for_autonomous_driving/actions/workflows/ci.yml)

该项目基于ROS2平台实现了自动驾驶中的简单定位功能，做这个项目的出发点源于本人在完成深蓝学院的[多传感器融合定位](https://www.shenlanxueyuan.com/course/558)课程后，有了对原始项目进行重构的想法，一方面可以回顾总结经验，另一方面可以锻炼工程能力，提高对定位工程的理解。由于以学习为目的，所以该项目偏重于代码的可读性以及扩展性，尽可能遵循ROS2的项目规范，以及代码风格，主要面向初学者学习入门。

本项目的主要框架以及核心代码参考了任乾老师的[从零开始做自动驾驶定位](https://github.com/Little-Potato-1990/localization_in_auto_driving)项目 ([知乎文章](https://zhuanlan.zhihu.com/p/113616755))。

## Quick start

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

## Plan

lidar定位

- [x] 实现lidar里程计功能: `lidar_odometry`
- [x] 实现基于lidar的建图功能: `lidar_mapping`
- [x] 实现基于lidar的定位功能: `lidar_localization`

lidar + imu 多传感器融合定位

- [ ] 实现imu里程计(常规积分，预积分): `imu_odometry`
- [x] 实现基于卡尔曼滤波的定位功能(eskf): `kf_based_localization`
- [x] 实现基于松耦合LIO的建图功能: `loosely_lio_mapping`
- [x] 实现基于图优化的定位功能(sliding window): `graph_based_localization`


## Acknowledgement

本项目参考了许多其它类似项目，并参考并引用了部分代码，这里向以下项目的作者表示感谢！

* https://github.com/Little-Potato-1990/localization_in_auto_driving
* https://github.com/AlexGeControl/Sensor-Fusion-for-Localization-Courseware
* https://github.com/gaoxiang12/slam_in_autonomous_driving
* https://github.com/HKUST-Aerial-Robotics/VINS-Mono
