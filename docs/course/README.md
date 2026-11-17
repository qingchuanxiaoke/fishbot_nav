# 基于 FishBot 的 ROS 2 移动机器人实践课程（8 × 90 分钟）

> 面向：人工智能专业大三学生（已具备 Python / C++ 基础、Linux 基本操作、线性代数与概率论基础）
> 平台：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + Nav2 + Cartographer
> 载体：本仓库 `fishbot_nav`（FishBot 两轮差速机器人的建图与导航上位机工程）

## 课程目标

学完本课程后，学生能够：

1. 熟练使用 ROS 2 的命令行、构建、可视化与调试工具（`colcon`、`ros2` CLI、`rqt`、RViz2、`tf2_tools`、`ros2 bag`）。
2. 读懂并编写 URDF/Xacro 机器人模型，在 Gazebo 中搭建带激光雷达的差速机器人仿真。
3. 使用 Cartographer 完成 2D SLAM 建图，并保存、加载栅格地图。
4. 理解 Nav2 的整体架构（生命周期节点、行为树、代价地图、AMCL），在仿真与实物上完成自主导航。
5. 能对比、替换和调优全局规划器（Planner）与局部控制器（Controller），并用 Python 编程调用导航功能。

## 工程结构与课程映射

| 功能包 | 内容 | 主要对应课次 |
|---|---|---|
| `fishbot_interfaces` | 自定义 `msg` / `srv`（`MyCustomMessage.msg`、`FishBotConfig.srv`） | 第 1 课 |
| `fishbot_bringup` | `odom2tf.cpp`（里程计话题→TF）、`urdf2tf` / `bringup` / `bringup_quick` 启动文件 | 第 2 课、第 8 课 |
| `fishbot_description` | `fishbot.urdf` 机器人模型 | 第 3、4 课 |
| `fishbot_cartographer` | Cartographer 配置 `fishbot_2d.lua`、启动文件、已建好的地图 | 第 5 课 |
| `fishbot_navigation2` | `nav2_params.yaml`、`navigation2.launch.py`、地图 `room.yaml` | 第 6、7、8 课 |
| `src/navigation`（子模块） | Nav2 源码（humble 分支），供阅读插件实现 | 第 7 课 |

> 说明：仓库本身面向实体 FishBot，不含 Gazebo 仿真包。第 3、4 课由学生在课上新建
> `fishbot_sim` 功能包（讲义中给出参考代码），此后第 5–7 课全部可以在仿真中完成，
> 第 8 课再迁移到实体机器人（如无实物，也可全程仿真）。

## 课时总览

| 课次 | 主题 | 关键词 | 课后作业 |
|---|---|---|---|
| [第 1 课](lesson01_ros2_tools.md) | ROS 2 开发环境与常用工具 | 工作空间、colcon、ros2 CLI、接口、rqt_graph | 自定义一个 srv 并用 CLI 调用 |
| [第 2 课](lesson02_tf_launch_debug.md) | 节点编程、TF 坐标变换与 Launch | `odom2tf.cpp`、tf2、view_frames、launch、ros2 bag | 用 Python 重写 odom2tf |
| [第 3 课](lesson03_urdf_modeling.md) | 机器人建模：URDF / Xacro | link/joint、惯性、碰撞、RViz2、check_urdf | 完成带轮子的 fishbot.xacro |
| [第 4 课](lesson04_gazebo_sim.md) | Gazebo 仿真：差速驱动与激光雷达 | gazebo_ros 插件、spawn_entity、world、teleop | 搭建自己的仿真场景 |
| [第 5 课](lesson05_slam_mapping.md) | SLAM 建图：Cartographer | lua 配置、子图/回环、map_saver | 提交地图 + 参数对比报告 |
| [第 6 课](lesson06_nav2_basics.md) | Nav2 自主导航入门 | 生命周期、BT、AMCL、代价地图、RViz 导航 | 代价地图参数调优记录 |
| [第 7 课](lesson07_planner_controller.md) | 使用不同规划器与控制器导航 | NavFn/A*、Smac、ThetaStar、DWB、RPP、MPPI | 规划器/控制器对比实验报告 |
| [第 8 课](lesson08_api_and_project.md) | 编程导航与综合项目（仿真→实物） | nav2_simple_commander、航点巡检、实物部署 | 综合项目演示与答辩 |

## 每节课的统一时间结构（90 分钟）

| 时段 | 时长 | 内容 |
|---|---|---|
| 回顾与导入 | 10 min | 检查上次作业、提出本课问题 |
| 原理讲解 | 20–25 min | 概念、架构图、对照本仓库代码讲解 |
| 实验操作 | 45–50 min | 按讲义分步完成，教师巡回答疑 |
| 总结与检查 | 5–10 min | 验收点检查、布置作业 |

## 考核建议

| 项目 | 占比 |
|---|---|
| 课堂验收点（每课 1–2 个，见各讲义"验收点"） | 30% |
| 课后作业（第 1–7 课） | 30% |
| 综合项目（第 8 课：演示 + 代码 + 报告） | 40% |

## 环境准备（课前由助教统一完成）

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cartographer-ros ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools ros-humble-rqt* ros-humble-nav2-simple-commander \
  ros-humble-turtlesim python3-colcon-common-extensions
```

```bash
mkdir -p ~/ws && cd ~/ws
git clone --recursive https://github.com/fishros/fishbot_nav.git -b humble
cd fishbot_nav
touch src/navigation/COLCON_IGNORE   # 子模块是 Nav2 完整源码，只读不编译
colcon build --symlink-install
```

> 子模块 `src/navigation` 是 Nav2 完整源码，全部编译需要很长时间。课程中使用 apt 安装的 Nav2，
> 子模块仅在第 7 课用于阅读规划器/控制器插件源码。
