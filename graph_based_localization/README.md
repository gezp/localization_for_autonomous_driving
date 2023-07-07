# graph_based_localization

运行demo

```bash
ros2 launch graph_based_localization sliding_window.launch.py 
```

* 可修改配置文件(`graph_based_localization/config/sliding_window.yaml`)，修改噪声项等参数
* 运行之前需要先进行建图，确保` ~/localization_data/map/` 下已经有地图文件。

保存轨迹

```bash
# kf_based_localizarion/scripts
bash save_odometry.sh
```

* 保存的目录在` ~/localization_data/trajectory/`

evo 轨迹评估

```bash
# kf_based_localizarion/scripts
bash evo_ape.sh fused_pose
bash evo_rpe.sh fused_pose
# 评价lidar pose, 与fused进行对比
bash evo_ape.sh lidar_pose
bash evo_rpe.sh lidar_pose
```
