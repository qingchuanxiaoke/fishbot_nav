# 第 5 课　SLAM 建图：Cartographer

**时长**：90 分钟　**对应代码**：`src/fishbot_cartographer/`（`config/fishbot_2d.lua`、`launch/cartographer.launch.py`、`map/`）

## 学习目标

- 理解 SLAM 问题：同时估计位姿与地图；前端（扫描匹配）与后端（位姿图优化、回环检测）。
- 读懂 `fishbot_2d.lua` 中每个修改过的参数的含义。
- 在仿真中完成建图，用 `map_saver_cli` 保存地图，读懂 `.pgm + .yaml` 地图格式。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾仿真；提问："只用里程计积分，绕一圈回来会怎样？"（现场演示漂移） |
| 10–35 | 讲解：占据栅格地图、Cartographer 子图与位姿图、回环检测；逐项解读 lua 配置 |
| 35–80 | 实验 1–4 |
| 80–90 | 总结、验收、作业 |

## 原理讲解要点

1. **占据栅格地图**：每个栅格的占据概率，`occupied_thresh / free_thresh` 三值化（对照 `room.yaml`：`resolution: 0.05`，`origin`，`mode: trinary`）。
2. **Cartographer 结构**：Local SLAM（扫描匹配 → 子图 Submap）+ Global SLAM（位姿图 + 回环约束，分支定界加速）。`cartographer_occupancy_grid_node` 将子图拼成 `/map`。
3. **`fishbot_2d.lua` 关键参数**（文件中已有中文注释，课上逐条讲）：

   | 参数 | 值 | 含义 |
   |---|---|---|
   | `tracking_frame` | `base_link` | SLAM 跟踪的坐标系 |
   | `published_frame` / `provide_odom_frame` | `odom` / `false` | 发布 `map→odom`，`odom→base` 由里程计提供 |
   | `use_odometry` | `true` | 融合里程计 |
   | `TRAJECTORY_BUILDER_2D.min_range / max_range` | 0.10 / 3.5 | 雷达有效距离 |
   | `use_online_correlative_scan_matching` | `true` | 前端用 CSM 提高鲁棒性（更耗 CPU） |
   | `motion_filter.max_angle_radians` | 0.1° | 运动滤波阈值，越小插入越频繁 |
   | `POSE_GRAPH.constraint_builder.min_score` | 0.65 | 回环约束接受阈值 |

4. **与 TF 的关系**：建图时 TF 链为 `map →(cartographer) odom →(里程计) base_footprint → base_link → laser_frame`。

## 实验步骤

### 实验 1：启动仿真与建图（15 min）

```bash
ros2 launch fishbot_sim sim.launch.py
ros2 launch fishbot_cartographer cartographer.launch.py      # 默认 use_sim_time=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

建图技巧：低速（线速度 ≤ 0.15 m/s）、转弯慢、**回到起点形成闭环**、对墙角多停留。RViz2 中观察 Submaps、Trajectory、Constraints 的变化。

### 实验 2：保存地图（5 min）

```bash
ros2 run nav2_map_server map_saver_cli -t map -f ~/ws/fishbot_nav/src/fishbot_navigation2/maps/sim_room
```

打开生成的 `sim_room.yaml` 与 `sim_room.pgm`（可用图片查看器/GIMP），讲解 `origin` 的含义。也可对比仓库中已有的 `fishbot_cartographer/map/fishbot_map.pgm`（实物建图结果）。

### 实验 3：参数对比实验（20 min）

分组修改以下参数之一，重新建图，对比地图质量（墙是否重影、是否闭合）：

- A 组：`use_odometry = false`
- B 组：`TRAJECTORY_BUILDER_2D.max_range = 8.0`
- C 组：`POSE_GRAPH.optimize_every_n_nodes = 0`（关闭全局优化）
- D 组：第 4 课雷达噪声 `stddev = 0.05`

> 用 `--symlink-install` 编译后修改 lua 无需重新编译，重启 launch 即可。

### 实验 4（选做）：离线建图

```bash
ros2 bag record /scan /odom /tf /tf_static /clock -o map_run
ros2 bag play map_run --clock
```

同一份数据用不同参数反复建图，排除"每次开车路线不同"的干扰——这是做对比实验的正确方法。

## 验收点

- [ ] 保存出一张闭合、无明显重影的仿真环境地图。
- [ ] 能解释 `.yaml` 中 `resolution`、`origin`、`occupied_thresh` 的含义。

## 课后作业

1. 提交自己场景的地图（`.pgm + .yaml`）。
2. 写一份 1 页的参数对比报告：本组参数修改前后的地图截图 + 原因分析。
3. （拓展）阅读 `slam_toolbox` 文档，简述它与 Cartographer 在使用上的差异。
