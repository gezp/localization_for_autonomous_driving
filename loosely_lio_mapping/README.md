# loosely_lio_mapping

运行demo

```bash
ros2 launch loosely_lio_mapping mapping.launch.py 
```

* 可参考`lidar_mapping`的建图流程，`lio_back_end`模块的配置文件(位于`loosely_lio_mapping/config/`中)

强制优化

```bash
# lidar_mapping/scripts
bash optimize_map.sh
```

保存scan_context

```bash
# lidar_mapping/scripts
bash save_scan_context.sh
```

* 保存的目录在` ~/localization_data/map/`

保存地图

```bash
# lidar_odometry/scripts
bash save_map.sh
```

* 保存的目录在` ~/localization_data/map/`

