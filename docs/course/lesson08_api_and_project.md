# 第 8 课　编程控制导航与综合项目（仿真 → 实物）

**时长**：90 分钟　**对应代码**：`fishbot_bringup/launch/bringup.launch.py`、`fishbot_navigation2/`；学生自建 `fishbot_tools`

## 学习目标

- 使用 `nav2_simple_commander`（Python API）编程实现单点导航、多航点巡检、任务取消与结果判断。
- 理解从仿真迁移到实物的关键差异：时间源、传感器 frame、通信链路（micro-ROS）、参数重新调优。
- 完成并演示综合项目。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾对比实验结论 |
| 10–25 | 讲解：Nav2 动作接口与 Simple Commander API；实物部署流程 |
| 25–55 | 实验 1–2：编写巡检程序，在仿真中跑通 |
| 55–80 | 实验 3：实物部署（或仿真中的综合项目） |
| 80–90 | 项目要求说明、评分标准 |

## 原理讲解要点

1. **Simple Commander 常用 API**：`setInitialPose()`、`waitUntilNav2Active()`、`goToPose()`、`goThroughPoses()`、`followWaypoints()`、`isTaskComplete()`、`getFeedback()`、`cancelTask()`、`getResult()`、`getPath()`。它是对 `NavigateToPose` 等动作客户端的封装。
2. **实物部署链路**（`bringup.launch.py`）：
   - `micro_ros_agent udp4 --port 8888`：FishBot 主控板通过 WiFi 发布 `/odom`、订阅 `/cmd_vel`。
   - `ros_serail2wifi tcp_server`：把雷达的 WiFi 数据转成虚拟串口 `/tmp/tty_laser`。
   - `ydlidar`：雷达驱动，延时 5 s 启动（等虚拟串口就绪）。
   - `urdf2tf` + `odom2tf`：静态 TF 与 `odom→base_footprint`。
   - 依赖的 `micro_ros_agent`、`ros_serail2wifi`、`ydlidar` 不在本仓库中，需按 FishBot 官方文档另行安装。
3. **仿真 → 实物检查清单**
   - 所有节点 `use_sim_time:=False`（导航启动时传参；`nav2_params.yaml` 中写死的 `True` 会被 `bringup_launch.py` 的参数覆盖；Cartographer 也要传 `use_sim_time:=false`）。
   - 雷达 `frame_id` 与 URDF 中的 `laser_frame` 一致；雷达安装方向（是否需要 yaw 180°）。
   - 实物速度、加速度上限更低；`robot_radius` 按实物外轮廓测量。
   - 网络：上位机与小车在同一局域网，`ROS_DOMAIN_ID` 一致。

## 实验步骤

### 实验 1：航点巡检程序（20 min）

```python
# fishbot_tools/fishbot_tools/patrol.py
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler   # sudo apt install ros-humble-tf-transformations

def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    q = quaternion_from_euler(0, 0, yaw)
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
    return p

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    points = [(1.0, 0.0, 0.0), (1.5, 1.0, 1.57), (0.0, 1.0, 3.14), (0.0, 0.0, 0.0)]  # 按地图修改
    for rnd in range(2):
        nav.followWaypoints([make_pose(nav, *pt) for pt in points])
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                nav.get_logger().info(f'第 {rnd+1} 圈，当前航点 {fb.current_waypoint + 1}/{len(points)}')
        result = nav.getResult()
        if result != TaskResult.SUCCEEDED:
            nav.get_logger().error(f'巡检失败: {result}')
            break
    nav.lifecycleShutdown()
    rclpy.shutdown()
```

### 实验 2：超时取消与异常处理（10 min）

改写：使用 `goToPose()` 逐点导航，若单个目标超过 30 s 未到达则 `cancelTask()` 并跳到下一个点；统计每个点的成功/失败并打印汇总表。

### 实验 3：实物部署（25 min，有实物时）

```bash
# 1. 启动底盘、雷达、TF
ros2 launch fishbot_bringup bringup.launch.py
# 2. 遥控确认 /odom、/scan、TF 正常
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 3. 实物建图
ros2 launch fishbot_cartographer cartographer.launch.py use_sim_time:=false
ros2 run nav2_map_server map_saver_cli -t map -f src/fishbot_navigation2/maps/lab
# 4. 实物导航
ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=False \
  map:=$HOME/ws/fishbot_nav/src/fishbot_navigation2/maps/lab.yaml
# 5. 运行巡检程序
ros2 run fishbot_tools patrol
```

**安全要求**：实物实验时一名同学手持遥控/随时准备关闭电源；首次运行把 `max_vel_x` 降到 0.15 m/s 以下。

无实物的班级：在仿真中加入动态障碍物（Gazebo 中移动的 actor 或脚本控制的箱子）完成同样的流程。

## 综合项目（第 8 课后 1–2 周内完成）

**题目（任选其一，或自拟后经教师同意）**

1. **室内巡检机器人**：在给定场景中按顺序巡视 ≥ 5 个点，每个点原地旋转一周"拍照"（订阅 `/scan` 记录最近障碍物距离），输出巡检报告。
2. **送物机器人**：通过自定义服务（扩展第 1 课 `fishbot_interfaces`）接收"送到房间 A/B/C"的请求，导航到对应位置并返回结果。
3. **算法对比研究**：在 3 种不同风格的场景（开阔、狭窄走廊、密集障碍）中系统对比 ≥ 3 种规划器 × ≥ 2 种控制器，给出选型建议。
4. **自定义插件（进阶）**：仿照 Nav2 插件接口，实现一个简单的全局规划器插件（如直线 + 碰撞检测，或自己实现 A*），在 `planner_server` 中加载使用。

**评分标准（综合项目 40 分）**

| 项目 | 分值 | 说明 |
|---|---|---|
| 功能完成度 | 15 | 现场演示（仿真或实物） |
| 代码质量 | 8 | 功能包结构、参数化、注释、能一键 launch |
| 实验与分析 | 10 | 数据、图表、结论是否有依据 |
| 报告与答辩 | 7 | 5 分钟展示 + 问答 |

## 课程总结

```
第1课 工具链 → 第2课 TF/Launch → 第3课 建模 → 第4课 仿真
      → 第5课 SLAM 建图 → 第6课 Nav2 导航 → 第7课 规划/控制算法 → 第8课 编程与实物
```

推荐延伸阅读：Nav2 官方文档（navigation.ros.org）中的 Tuning Guide 与 Plugin Tutorials；Cartographer 官方调参文档；REP-103 / REP-105。
