# 第 6 课　Nav2 自主导航入门

**时长**：90 分钟　**对应代码**：`src/fishbot_navigation2/`（`launch/navigation2.launch.py`、`config/nav2_params.yaml`、`maps/`）

## 学习目标

- 掌握 Nav2 的整体架构与各服务器节点的职责。
- 理解生命周期节点（Lifecycle Node）与行为树（Behavior Tree）在 Nav2 中的作用。
- 理解 AMCL 粒子滤波定位与代价地图（静态层、障碍层/体素层、膨胀层）。
- 在仿真中使用自己建的地图完成"初始定位 → 单点导航 → 多点导航"。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾建图；提问："有了地图，机器人怎么知道自己在哪？怎么走过去？" |
| 10–35 | 讲解：Nav2 架构、AMCL、代价地图、BT Navigator |
| 35–80 | 实验 1–4 |
| 80–90 | 总结、验收、作业 |

## 原理讲解要点

1. **Nav2 架构**（对照 `nav2_params.yaml` 的顶层键）

   | 节点 | 职责 | 本工程的配置 |
   |---|---|---|
   | `map_server` | 加载静态地图 | 默认 `maps/room.yaml` |
   | `amcl` | 定位，发布 `map→odom` | 差速运动模型，500–2000 粒子，`likelihood_field` |
   | `bt_navigator` | 用行为树编排整个导航流程 | 默认 `navigate_to_pose_w_replanning_and_recovery.xml` |
   | `planner_server` | 全局路径规划 | `NavfnPlanner`（`use_astar: false` 即 Dijkstra） |
   | `controller_server` | 局部轨迹跟踪，输出 `/cmd_vel` | `DWBLocalPlanner`，20 Hz |
   | `smoother_server` | 路径平滑 | `SimpleSmoother` |
   | `behavior_server` | 恢复行为 | spin / backup / drive_on_heading / wait |
   | `waypoint_follower` | 多航点 | `WaitAtWaypoint` |
   | `velocity_smoother` | 速度平滑 | `max_velocity: [0.26, 0.0, 1.0]` |

2. **生命周期节点**：`unconfigured → inactive → active`，由 `lifecycle_manager` 统一管理；可用 `ros2 lifecycle get /amcl` 查看。
3. **AMCL**：粒子滤波 = 运动模型预测（`alpha1–5` 为里程计噪声）+ 观测模型更新（似然场）+ 重采样。需要给出初始位姿。
4. **代价地图**：全局代价地图（`static + obstacle + inflation`，`map` 坐标系）与局部代价地图（`voxel + inflation`，`odom` 坐标系、3 m × 3 m 滚动窗口）。讲清 `robot_radius`、`inflation_radius`、`cost_scaling_factor` 的几何意义。
5. **讨论点**：本工程 `robot_radius: 0.22`，但 URDF 中底盘半径只有 0.10 m——这是保守设置，代价是窄通道过不去。下面的实验会调整它。

## 实验步骤

### 实验 1：启动导航（10 min）

```bash
ros2 launch fishbot_sim sim.launch.py
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$HOME/ws/fishbot_nav/src/fishbot_navigation2/maps/sim_room.yaml
```

> `navigation2.launch.py` 通过 `IncludeLaunchDescription` 复用了 `nav2_bringup` 的 `bringup_launch.py`，只替换了地图和参数文件——这是在自己的工程中使用 Nav2 的推荐方式。

### 实验 2：定位与单点导航（10 min）

1. RViz2 中点 **2D Pose Estimate**，在地图上给出机器人当前位置和朝向；观察粒子云（`/particle_cloud`）收敛。
2. 点 **Nav2 Goal** 设置目标；观察全局路径（`/plan`）、局部路径、代价地图。
3. 命令行观察：

```bash
ros2 topic echo /amcl_pose --once
ros2 action list
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}" --feedback
```

### 实验 3：多点导航与恢复行为（10 min）

- RViz2 Nav2 面板中切换到 **Waypoint / Nav Through Poses Mode**，设置 3–4 个航点后执行。
- 在 Gazebo 中，把一个箱子拖到机器人正在走的路径上，观察重规划；把机器人"围死"，观察 spin/backup 等恢复行为。

### 实验 4：代价地图参数调优（15 min）

在线修改参数并观察 RViz2 中代价地图的变化：

```bash
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.30
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 5.0
ros2 param set /global_costmap/global_costmap robot_radius 0.12
rqt   # Plugins → Configuration → Dynamic Reconfigure / Parameter Reconfigure
```

设计一条宽约 0.4 m 的窄通道，找到能让机器人"既能通过又不蹭墙"的参数组合，然后写回 `nav2_params.yaml`。

## 验收点

- [ ] 在自建地图上完成初始定位并成功导航到 3 个不同目标点。
- [ ] 能说出 `robot_radius`、`inflation_radius`、`cost_scaling_factor` 分别如何影响路径。

## 课后作业

1. 提交代价地图调参记录（参数、截图、结论）。
2. 打开 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`，用 Groot 或手绘方式画出行为树结构，并说明 `RecoveryNode`、`PipelineSequence`、`RateController` 的作用。
