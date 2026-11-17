# 第 2 课　节点编程、TF 坐标变换与 Launch 启动

**时长**：90 分钟　**对应代码**：`src/fishbot_bringup/src/odom2tf.cpp`、`src/fishbot_bringup/launch/*.launch.py`

## 学习目标

- 读懂 C++ 节点 `odom2tf`：订阅、回调、QoS、TF 广播。
- 理解移动机器人的标准坐标系树 `map → odom → base_footprint → base_link → laser_frame`（REP-105）。
- 会使用 TF 调试工具：`view_frames`、`tf2_echo`、RViz2 的 TF 显示。
- 会读写 Python Launch 文件；会用 `ros2 bag` 录制与回放数据。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾第 1 课作业；提问"雷达看到的障碍物，如何换算到地图上？" |
| 10–35 | 讲解：坐标系与 TF；逐行解读 `odom2tf.cpp`；Launch 文件结构 |
| 35–80 | 实验 1–4 |
| 80–90 | 总结、验收、作业 |

## 原理讲解要点

1. **REP-105 坐标系约定**
   - `map`：全局固定，长期无漂移但可能跳变（由 AMCL/SLAM 发布 `map→odom`）。
   - `odom`：连续平滑但会累积漂移（由里程计发布 `odom→base_footprint`）。
   - `base_footprint`/`base_link`/`laser_frame`：机器人本体，静态关系由 URDF + `robot_state_publisher` 发布。
2. **`odom2tf.cpp` 的作用**：FishBot 下位机（micro-ROS）只发布 `/odom` 话题，不发布 TF；该节点把 `nav_msgs/Odometry` 的位姿原样转成 `odom→base_footprint` 变换。重点讲：
   - `rclcpp::SensorDataQoS()`：Best-Effort，与 micro-ROS 发布端匹配；QoS 不匹配时"有话题但收不到数据"。
   - `transform.header = msg->header`：时间戳必须用数据本身的时间，而不是 `now()`。
   - `child_frame_id` 依赖固件设置（代码注释提到 1.7.0 之前的固件需要改成 `base_footprint`）。
3. **Launch 文件**：对比 `bringup_quick.launch.py`（URDF→TF + odom2tf）和 `bringup.launch.py`（再加上 micro-ROS Agent、串口转 WiFi、YDLidar 驱动，并用 `TimerAction` 延时 5 s 启动雷达）。讲 `DeclareLaunchArgument`、`LaunchConfiguration`、`IncludeLaunchDescription`、`Command(['cat ', ...])`。
4. **工程细节讨论**：`fishbot_bringup/package.xml` 中只声明了构建工具依赖，没有声明 `rclcpp`、`tf2_ros` 等运行依赖——请学生思考这对 `rosdep install` 会有什么影响，并在作业中补全。

## 实验步骤

### 实验 1：手动发布 odom，观察 TF（15 min）

```bash
ros2 launch fishbot_bringup bringup_quick.launch.py
```

另开终端模拟下位机发布里程计（注意 QoS 设为 best_effort）：

```bash
ros2 topic pub -r 10 --qos-reliability best_effort /odom nav_msgs/msg/Odometry \
"{header: {frame_id: 'odom'}, child_frame_id: 'base_footprint', pose: {pose: {position: {x: 1.0, y: 0.5}, orientation: {z: 0.383, w: 0.924}}}}"
```

```bash
ros2 run tf2_tools view_frames            # 生成 frames.pdf
ros2 run tf2_ros tf2_echo odom laser_frame
rviz2                                      # Fixed Frame 选 odom，添加 TF、RobotModel
```

思考：把 `--qos-reliability` 改成 `reliable` 后还能收到吗？为什么？（`ros2 topic info /odom -v` 查看两端 QoS）

### 实验 2：rqt 工具箱（10 min）

- `rqt_graph`：观察 `odom2tf`、`robot_state_publisher`、`joint_state_publisher` 的连接。
- `rqt_tf_tree`：图形化 TF 树。
- `rqt_console`：按日志级别过滤输出。
- `ros2 doctor --report`：检查环境。

### 实验 3：修改 Launch 文件（10 min）

给 `bringup_quick.launch.py` 增加一个 `use_rviz` 参数，为 `true` 时同时启动 RViz2。要点：`DeclareLaunchArgument` + `IfCondition(LaunchConfiguration('use_rviz'))`。

### 实验 4：ros2 bag 录制与回放（10 min）

```bash
ros2 bag record /odom /tf /tf_static -o odom_demo
ros2 bag info odom_demo
ros2 bag play odom_demo --rate 2.0
```

说明：后续第 5 课可以录制激光+里程计数据，离线反复跑 SLAM 调参。

## 验收点

- [ ] `view_frames` 生成的 TF 树包含 `odom → base_footprint → base_link → laser_frame / laser_link`。
- [ ] Launch 文件新增的 `use_rviz` 参数可用。

## 课后作业

1. 用 Python（rclpy + `tf2_ros.TransformBroadcaster`）重写 `odom2tf`，加入参数 `child_frame_override`（非空时覆盖 `child_frame_id`）。
2. 补全 `fishbot_bringup/package.xml` 中缺失的依赖，说明每一条的作用。
