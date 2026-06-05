# Mission Logic 启动说明

本文档只说明新增 `mission_logic` 的启动方式，不替代原有 README。

## 1. 编译和加载环境

```bash
cd /home/white/pipe_tot/cable_detection_development_environment
colcon build --symlink-install
source install/setup.bash
```

如果打开了新终端，需要重新执行：

```bash
source /home/white/pipe_tot/cable_detection_development_environment/install/setup.bash
```

## 2. 仿真启动

终端 1 启动原仿真系统，例如：

```bash
ros2 launch vehicle_simulator system_garage.launch
```

终端 2 启动任务逻辑和仿真磁场：

```bash
ros2 launch mission_logic mission_logic.launch.py
```

默认会启动：

```text
magnetic_field_node  -> 发布 /magnetic_field 和 /pipeline_marker
mission_node         -> 订阅 /state_estimation, /magnetic_field，发布 /goal_pose, /speed
```

如果要指定其他管线 JSON：

```bash
ros2 launch mission_logic mission_logic.launch.py pipeline_config_file:=/path/to/pipeline.json
```

## 3. 真机启动

真机入口仍使用原工程的 real robot launch：

```bash
ros2 launch vehicle_simulator system_real_robot.launch
```

默认不会启动 `mission_logic`，因此原行为保持不变。需要接入任务逻辑时：

```bash
ros2 launch vehicle_simulator system_real_robot.launch start_mission_logic:=true
```

此时只启动 `mission_node`，不会启动仿真的 `magnetic_field_node`。真机上需要由真实磁传感器节点发布：

```text
/magnetic_field    sensor_msgs/msg/MagneticField
```

真机链路应为：

```text
真机 SLAM/LOAM -> loamInterface -> /state_estimation, /registered_scan
真实磁传感器   -> /magnetic_field
mission_node   -> /goal_pose, /speed
localPlanner/pathFollower -> /cmd_vel
底盘驱动       -> 执行动作
```

## 4. 常用检查命令

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /magnetic_field
ros2 topic echo /goal_pose
ros2 topic echo /cmd_vel
```

RViz 中可查看：

```text
/goal_pose
/path
/terrain_map
/registered_scan
/pipeline_marker
```

