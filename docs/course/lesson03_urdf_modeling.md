# 第 3 课　机器人建模：URDF 与 Xacro

**时长**：90 分钟　**对应代码**：`src/fishbot_description/urdf/fishbot.urdf`、`src/fishbot_bringup/launch/urdf2tf.launch.py`

## 学习目标

- 掌握 URDF 的 `link`（visual / collision / inertial）与 `joint`（fixed / continuous / revolute）。
- 会用 Xacro 的属性、数学表达式和宏简化模型。
- 会用 `check_urdf`、`urdf_to_graphviz`、`joint_state_publisher_gui` + RViz2 检查模型。
- 为下节课仿真准备一个"物理完整"的 FishBot 模型。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 回顾 TF；提问："TF 树里静态的那部分是谁发布的？" |
| 10–30 | 讲解：URDF 语法；分析现有 `fishbot.urdf` 的不足 |
| 30–80 | 实验 1–3：从 URDF 到 Xacro，补全轮子、惯性与碰撞 |
| 80–90 | 总结、验收、作业 |

## 原理讲解要点

1. **现有模型分析**（`fishbot.urdf`）：
   - `base_footprint`（地面投影）→ `base_link`（圆柱 r=0.10 m, h=0.12 m，抬高 0.076 m）→ `laser_link` / `laser_frame`（高 0.075 m）。
   - 只有 `visual`，没有 `collision` 和 `inertial`，也没有轮子——**足够用于实物导航的 TF，但不能用于物理仿真**。
   - 为什么有两个雷达 link？（兼容不同雷达驱动的 `frame_id`）引出"frame_id 必须与传感器消息一致"。
2. **惯性矩阵**：圆柱体 `ixx = iyy = m(3r²+h²)/12`，`izz = mr²/2`；长方体、球体公式。惯性错误会导致 Gazebo 中机器人"飞走"或抖动。
3. **差速底盘结构**：两个主动轮（`continuous` 关节，绕 y 轴）+ 万向轮（简化为摩擦很小的球体，`fixed`）。
4. **Xacro**：`xacro:property`、`${}` 表达式、`xacro:macro` 复用左右轮、`xacro:include` 拆分文件。

## 实验步骤

### 实验 1：可视化与检查现有模型（10 min）

```bash
sudo apt install liburdfdom-tools
check_urdf src/fishbot_description/urdf/fishbot.urdf
urdf_to_graphviz src/fishbot_description/urdf/fishbot.urdf
ros2 launch fishbot_bringup urdf2tf.launch.py
rviz2   # Fixed Frame: base_footprint，添加 RobotModel（Description Topic: /robot_description）和 TF
```

### 实验 2：改写为 Xacro 并加入轮子（30 min）

在 `fishbot_description/urdf/` 下新建 `fishbot.urdf.xacro`。参考结构（尺寸可让学生实测实物后修改）：

```xml
<?xml version="1.0"?>
<robot name="fishbot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base_r" value="0.10"/>
  <xacro:property name="base_h" value="0.12"/>
  <xacro:property name="wheel_r" value="0.032"/>
  <xacro:property name="wheel_w" value="0.04"/>
  <xacro:property name="wheel_y" value="0.10"/>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
               iyy="${m*(3*r*r+h*h)/12}" iyz="0" izz="${m*r*r/2}"/>
    </inertial>
  </xacro:macro>

  <link name="base_footprint"/>
  <link name="base_link">
    <visual><geometry><cylinder radius="${base_r}" length="${base_h}"/></geometry></visual>
    <collision><geometry><cylinder radius="${base_r}" length="${base_h}"/></geometry></collision>
    <xacro:cylinder_inertia m="1.0" r="${base_r}" h="${base_h}"/>
  </link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.076"/>
  </joint>

  <xacro:macro name="wheel" params="side y">
    <link name="${side}_wheel_link">
      <visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry></visual>
      <collision><origin rpy="1.5708 0 0"/><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry></collision>
      <xacro:cylinder_inertia m="0.1" r="${wheel_r}" h="${wheel_w}"/>
    </link>
    <joint name="${side}_wheel_joint" type="continuous">
      <parent link="base_link"/><child link="${side}_wheel_link"/>
      <origin xyz="0 ${y} ${wheel_r-0.076}"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  <xacro:wheel side="left"  y="${wheel_y}"/>
  <xacro:wheel side="right" y="${-wheel_y}"/>

  <!-- 万向轮：前后各一个小球，摩擦在第 4 课的 gazebo 标签中设为 0 -->
  <!-- laser_frame：沿用原 URDF 的位置 (0 0 0.075) -->
</robot>
```

由学生补全：前后万向轮（球体，半径 = `wheel_r/2`，z 方向保证与地面接触）、`laser_frame` 及其惯性。

修改 `urdf2tf.launch.py`，把 `Command(['cat ', ...])` 换成 `Command(['xacro ', ...])`，并将默认模型路径改为 `.xacro` 文件。

### 实验 3：用 GUI 转动轮子（10 min）

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

拖动滑块，在 RViz2 中检查左右轮转轴方向是否正确（正值应使机器人前进）。

## 验收点

- [ ] `check_urdf` 无报错，树结构包含左右轮、万向轮、雷达。
- [ ] RViz2 中模型外观正确，轮子贴地，滑块能转动轮子。

## 课后作业

1. 完成 `fishbot.urdf.xacro`，把材质颜色、雷达、万向轮拆成单独的 `.xacro` 文件并 `include`。
2. 简答：`base_footprint` 与 `base_link` 为什么要分开？Nav2 参数中哪些地方用到了它们？（提示：查看 `nav2_params.yaml` 中 `amcl.base_frame_id` 与 `robot_base_frame`）
