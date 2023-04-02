# lidar_odometry

运行demo

```bash
ros2 launch lidar_odometry front_end.launch.py 
```

* 可修改配置文件(`lidar_odometry/config/front_end_config.yaml`)，选择不同点云匹配方法：`ICP`, `ICP_SVD`, `NDT`, `NDT_OMP`

保存轨迹

```bash
# lidar_odometry/scripts
bash save_odometry.sh
```

* 保存的目录在` ~/localization_data/trajectory/`

evo 轨迹评估

```bash
# lidar_odometry/scripts
bash evo_ape.sh
bash evo_rpe.sh
```
