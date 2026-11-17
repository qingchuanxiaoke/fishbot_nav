# 第 7 课　使用不同的规划器与控制器导航

**时长**：90 分钟　**对应代码**：`nav2_params.yaml` 中 `planner_server`、`controller_server` 段；子模块 `src/navigation` 中的插件源码

## 学习目标

- 理解 Nav2 的插件机制（pluginlib）：规划器、控制器、代价地图层、恢复行为都是可替换的插件。
- 掌握主流全局规划器：NavFn（Dijkstra / A*）、Smac 2D、Smac Hybrid-A*、Theta*。
- 掌握主流局部控制器：DWB（动态窗口）、Regulated Pure Pursuit（RPP）、MPPI（模型预测路径积分）。
- 用统一的实验方法对比不同算法，并根据场景选型。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾 Nav2 架构；展示"同一起终点，不同规划器路径不一样" |
| 10–35 | 讲解：规划器与控制器算法原理（结合 AI 课程中的搜索与优化知识） |
| 35–80 | 实验 1–3 |
| 80–90 | 总结、选型建议、布置对比实验报告 |

## 原理讲解要点

### 全局规划器（Planner）

| 插件 | 算法 | 特点 | 适用 |
|---|---|---|---|
| `nav2_navfn_planner/NavfnPlanner` | Dijkstra / A*（`use_astar`） | 基于势场的波前传播，稳定 | 圆形差速机器人（本工程默认） |
| `nav2_smac_planner/SmacPlanner2D` | 8 连通 A* + 代价感知启发 | 路径更贴近代价最优 | 圆形机器人 |
| `nav2_smac_planner/SmacPlannerHybrid` | Hybrid-A*（Dubins/Reeds-Shepp） | 考虑最小转弯半径，可倒车 | 阿克曼/大型机器人 |
| `nav2_theta_star_planner/ThetaStarPlanner` | Theta*（任意角度） | 折线少、路径短直 | 开阔场景 |

联系 AI 课程：A* 的可采纳启发函数、Dijkstra 是 h=0 的 A*、状态空间设计（Hybrid-A* 把朝向加入状态）。

### 局部控制器（Controller）

| 插件 | 原理 | 特点 |
|---|---|---|
| `dwb_core::DWBLocalPlanner` | 速度空间采样 + 前向仿真 + critics 加权打分 | 可解释、可调项多（本工程默认） |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 纯跟踪几何法 + 曲率/接近障碍物时自动降速 | 参数少、平滑、计算量小 |
| `nav2_mppi_controller::MPPIController` | 随机采样控制序列 + 代价加权平均（采样式 MPC） | 动态避障好，计算量较大 |

**DWB critics 讲解**（对照本工程配置）：`BaseObstacle`（离障碍物）、`PathAlign`/`PathDist`（贴合全局路径）、`GoalAlign`/`GoalDist`（朝向目标）、`RotateToGoal`（到点后原地转向）、`Oscillation`（防止来回振荡）。总代价 = Σ scale × critic 得分。

### 其他相关插件

- 目标检查器 `SimpleGoalChecker`（`xy_goal_tolerance: 0.25`、`yaw_goal_tolerance: 0.25`）；进度检查器 `SimpleProgressChecker`（10 s 内至少移动 0.5 m）。

## 实验步骤

### 实验 1：同时加载多个规划器（15 min）

修改 `nav2_params.yaml` 的 `planner_server`：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased", "AStar", "Smac2D", "ThetaStar"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    AStar:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
    Smac2D:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.25
      allow_unknown: true
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
    ThetaStar:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
```

用脚本对同一起终点分别调用 4 个规划器，记录路径长度与耗时（脚本放在 `fishbot_tools` 包中）：

```python
# compare_planners.py
import math, time, rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def pose(nav, x, y):
    p = PoseStamped(); p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p

def length(path):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

rclpy.init()
nav = BasicNavigator()
nav.waitUntilNav2Active()
start, goal = pose(nav, 0.0, 0.0), pose(nav, 3.0, 1.5)   # 按自己的地图修改
for pid in ['GridBased', 'AStar', 'Smac2D', 'ThetaStar']:
    t = time.time()
    path = nav.getPath(start, goal, planner_id=pid, use_start=True)
    dt = (time.time() - t) * 1000
    print(f'{pid:10s} len={length(path):.2f} m  poses={len(path.poses)}  time={dt:.0f} ms' if path else f'{pid}: 失败')
rclpy.shutdown()
```

在 RViz2 中添加 `/plan` 显示，对比几条路径的形状。

> 注意：行为树默认使用 `planner_id` 为 `GridBased` 的规划器；若想让 RViz2 的 Nav2 Goal 使用其他规划器，最简单的方法是把它放在 `planner_plugins` 里并命名为 `GridBased`。

### 实验 2：替换控制器（20 min）

分组把 `controller_server` 的 `FollowPath` 替换为以下之一（保留其余参数），在同一条路线上导航并录包：

**RPP：**

```yaml
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.2
      lookahead_dist: 0.4
      min_lookahead_dist: 0.2
      max_lookahead_dist: 0.8
      use_velocity_scaled_lookahead_dist: true
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.6
      regulated_linear_scaling_min_speed: 0.05
      use_rotate_to_heading: true
      rotate_to_heading_angular_vel: 1.0
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      allow_reversing: false
      transform_tolerance: 0.2
```

**MPPI：**

```yaml
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 1000
      iteration_count: 1
      vx_max: 0.26
      vx_min: -0.1
      wz_max: 1.0
      motion_model: "DiffDrive"
      critics: ["ConstraintCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic", "ObstaclesCritic"]
```

> `model_dt` 应与 `controller_frequency` 对应（20 Hz → 0.05 s）。MPPI 在 Humble 较新的二进制包中提供，若 `ros2 pkg list | grep mppi` 为空则跳过该组。

记录指标：

```bash
ros2 bag record /cmd_vel /odom /plan /local_plan -o ctrl_<组名>
```

| 指标 | 获取方式 |
|---|---|
| 到达时间 | 动作反馈 `navigation_time` |
| 轨迹平滑度 | `/cmd_vel` 角速度的方差 / 跳变次数（`rqt_plot`） |
| 跟踪误差 | `/odom` 轨迹与 `/plan` 的平均距离 |
| 是否碰撞 / 恢复次数 | 动作反馈 `number_of_recoveries` |

### 实验 3：DWB critics 调参（10 min，DWB 组）

```bash
ros2 param set /controller_server FollowPath.PathDist.scale 64.0
ros2 param set /controller_server FollowPath.BaseObstacle.scale 0.5
ros2 param set /controller_server FollowPath.max_vel_x 0.15
```

观察机器人"贴路径"与"躲障碍"之间的取舍。

### 拓展：阅读插件源码

- `src/navigation/nav2_navfn_planner/src/navfn.cpp`：Dijkstra/A* 势场传播。
- `src/navigation/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`：`computeVelocityCommands()` 中的前视点与曲率计算。
- `src/navigation/nav2_dwb_controller/dwb_critics/`：各 critic 的打分函数。

## 验收点

- [ ] 能用脚本输出 4 个规划器的路径长度与耗时。
- [ ] 至少用 2 种控制器完成同一路线的导航。

## 课后作业：规划器与控制器对比实验报告

要求：同一地图、同一组起终点（≥ 3 组）、每组至少重复 3 次；给出表格与图（路径叠加图、速度曲线），并回答：

1. 哪个规划器路径最短？哪个最快？为什么 Theta* 的路径折点少？
2. DWB / RPP /（MPPI）在窄通道、急转弯、动态障碍物三种场景下各自表现如何？
3. 如果 FishBot 换成阿克曼转向底盘，规划器和控制器应如何选型？
