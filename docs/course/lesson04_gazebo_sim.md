# 第 4 课　Gazebo 仿真：差速驱动与激光雷达

**时长**：90 分钟　**对应代码**：第 3 课完成的 `fishbot.urdf.xacro`；新建功能包 `fishbot_sim`

## 学习目标

- 理解 Gazebo 与 ROS 2 的桥接方式（`gazebo_ros` 插件）以及仿真时间 `/clock`、`use_sim_time`。
- 为模型添加差速驱动插件与激光雷达插件，使仿真机器人与实物 FishBot 拥有**相同的话题和 TF 接口**（`/cmd_vel`、`/odom`、`/scan`、`odom→base_footprint`）。
- 搭建仿真世界，用键盘遥控机器人并在 RViz2 中查看雷达数据。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾 URDF；讨论"仿真与实物的接口一致性"为什么重要 |
| 10–30 | 讲解：Gazebo 架构、插件机制、摩擦/惯性、仿真时间 |
| 30–80 | 实验 1–4 |
| 80–90 | 总结、验收、作业 |

## 原理讲解要点

1. **实物 vs 仿真数据来源对比**

   | 接口 | 实物 FishBot（`bringup.launch.py`） | 仿真 |
   |---|---|---|
   | `/cmd_vel` → 电机 | micro-ROS 下位机 | `gazebo_ros_diff_drive` |
   | `/odom` | micro-ROS 下位机 | `gazebo_ros_diff_drive` |
   | `odom→base_footprint` TF | `odom2tf` 节点 | `gazebo_ros_diff_drive`（`publish_odom_tf`） |
   | `/scan` | YDLidar 驱动 | `gazebo_ros_ray_sensor` |
   | 静态 TF | `robot_state_publisher` | `robot_state_publisher`（`use_sim_time:=true`） |

   **注意**：仿真中不要再启动 `odom2tf`，否则会有两个节点同时发布 `odom→base_footprint`。
2. **仿真时间**：Gazebo 发布 `/clock`，所有节点需 `use_sim_time:=true`。本仓库 `nav2_params.yaml` 和 `cartographer.launch.py` 默认就是 `true`，说明它们原本即可用于仿真。
3. **物理参数**：万向轮 `mu1/mu2 = 0`；驱动轮摩擦足够大；惯性合理。

## 实验步骤

### 实验 1：创建 fishbot_sim 功能包（5 min）

```bash
cd ~/ws/fishbot_nav/src
ros2 pkg create fishbot_sim --build-type ament_cmake
mkdir -p fishbot_sim/{launch,worlds,urdf}
```

在 `CMakeLists.txt` 中 `install(DIRECTORY launch worlds urdf DESTINATION share/${PROJECT_NAME})`。

### 实验 2：编写 Gazebo 插件描述（20 min）

新建 `fishbot_sim/urdf/fishbot.gazebo.xacro`，并在主模型中 `include`：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <gazebo reference="front_caster_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="back_caster_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>

  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.20</wheel_separation>
      <wheel_diameter>0.064</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <gazebo reference="laser_frame">
    <sensor name="laser" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan><horizontal><samples>360</samples><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
        <range><min>0.12</min><max>8.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </ray>
      <plugin name="laser_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

> `wheel_separation` 应等于左右轮中心距（第 3 课 `wheel_y` 的两倍），`wheel_diameter = 2 × wheel_r`。

### 实验 3：世界文件与启动文件（15 min）

`worlds/room.world`：用 Gazebo 的 Building Editor 画一个约 6 m × 4 m 的房间（含几个障碍物），保存为 world 文件。

`launch/sim.launch.py` 关键内容：

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc = get_package_share_directory('fishbot_description')
    sim = get_package_share_directory('fishbot_sim')
    xacro_file = os.path.join(desc, 'urdf', 'fishbot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(sim, 'worlds', 'room.world')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'fishbot', '-z', '0.01'])

    return LaunchDescription([gazebo, rsp, spawn])
```

### 实验 4：运行与遥控（10 min）

```bash
colcon build --packages-select fishbot_description fishbot_sim && source install/setup.bash
ros2 launch fishbot_sim sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

检查清单：

```bash
ros2 topic hz /scan            # 约 10 Hz
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
rviz2   # Fixed Frame: odom；添加 RobotModel、LaserScan(/scan)、Odometry(/odom)
```

## 验收点

- [ ] 键盘遥控时，Gazebo 与 RViz2 中机器人同步运动，前进/转向方向正确。
- [ ] RViz2 中激光点云与 Gazebo 中墙体位置吻合。

## 课后作业

1. 搭建一个自己设计的场景（至少包含一条走廊、一扇"门"、3 个以上障碍物），提交 world 文件与截图。
2. 把雷达噪声 `stddev` 分别设为 0.01 和 0.05，观察 `/scan` 在 RViz2 中的差异，为下节课建图做准备。

## 常见问题

- 机器人在 Gazebo 中翻倒/滑动：检查惯性、万向轮高度与摩擦。
- RViz2 中 TF 报 "extrapolation into the future"：某个节点没有设置 `use_sim_time:=true`。
- Gazebo 首次启动很慢：在下载模型库，可提前缓存 `~/.gazebo/models`。
