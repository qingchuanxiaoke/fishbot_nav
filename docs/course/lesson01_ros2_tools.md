# 第 1 课　ROS 2 开发环境与常用工具

**时长**：90 分钟　**对应代码**：整个工作空间结构、`src/fishbot_interfaces`

## 学习目标

- 理解 ROS 2 的核心概念：节点、话题、服务、参数、动作、接口（msg/srv/action）、DDS 与 QoS。
- 会用 `colcon` 构建工作空间，理解 `src/ build/ install/ log/` 与 `source install/setup.bash`。
- 熟练使用 `ros2` 命令行工具与 `rqt_graph` 观察系统。
- 读懂并编译 `fishbot_interfaces` 中的自定义接口。

## 课堂安排

| 时间 | 内容 |
|---|---|
| 0–10 | 课程介绍：FishBot 是什么、最终要做到什么（播放仿真自主导航演示） |
| 10–35 | 讲解：ROS 2 通信模型；ament_cmake 功能包结构；本仓库 5 个功能包分别负责什么 |
| 35–80 | 实验 1–4 |
| 80–90 | 总结、验收、布置作业 |

## 原理讲解要点

1. **ROS 2 vs ROS 1**：无 master，基于 DDS 自动发现；`ROS_DOMAIN_ID` 隔离实验室中多组同学的网络（**课上每组设置不同的 ID，否则会互相串话题**）。
2. **四种通信方式**：话题（异步、多对多）、服务（同步请求-响应）、动作（长任务 + 反馈 + 可取消，Nav2 的 `NavigateToPose` 就是动作）、参数。
3. **功能包结构**：`package.xml`（依赖声明）+ `CMakeLists.txt`（构建规则）。以 `fishbot_interfaces` 为例讲 `rosidl_generate_interfaces`。
4. **工作空间叠加（overlay）**：`/opt/ros/humble` 是 underlay，`~/ws/fishbot_nav/install` 是 overlay。

## 实验步骤

### 实验 1：环境检查与 turtlesim 热身（10 min）

```bash
echo "export ROS_DOMAIN_ID=<组号>" >> ~/.bashrc && source ~/.bashrc
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key        # 新终端
```

用以下命令观察系统：

```bash
ros2 node list
ros2 node info /turtlesim
ros2 topic list -t
ros2 topic echo /turtle1/pose
ros2 topic hz /turtle1/pose
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.8}}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.0, name: 't2'}"
ros2 param list /turtlesim
ros2 param set /turtlesim background_r 200
rqt_graph
```

### 实验 2：编译 FishBot 工作空间（10 min）

```bash
cd ~/ws/fishbot_nav
touch src/navigation/COLCON_IGNORE
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep fishbot
```

讨论：`--symlink-install` 的作用（修改 launch/yaml 后无需重新编译）；`colcon build --packages-select fishbot_interfaces` 单独编译。

### 实验 3：查看自定义接口（10 min）

```bash
ros2 interface show fishbot_interfaces/msg/MyCustomMessage
ros2 interface show fishbot_interfaces/srv/FishBotConfig
ros2 interface proto fishbot_interfaces/msg/MyCustomMessage
ros2 topic pub --once /test fishbot_interfaces/msg/MyCustomMessage "{int32_test: 42, float64_test: 3.14}"
```

对照 `src/fishbot_interfaces/CMakeLists.txt` 讲解 `.msg` 中的类型如何映射到 C++/Python 类型（`byte`、`char` 的特殊之处）。

### 实验 4：写一个最小的服务端（15 min）

新建 Python 包 `fishbot_tools`，实现 `FishBotConfig` 服务：收到 `key/value` 后存入字典并原样返回。

```bash
cd ~/ws/fishbot_nav/src
ros2 pkg create fishbot_tools --build-type ament_python --dependencies rclpy fishbot_interfaces
```

```python
# fishbot_tools/fishbot_tools/config_server.py
import rclpy
from rclpy.node import Node
from fishbot_interfaces.srv import FishBotConfig

class ConfigServer(Node):
    def __init__(self):
        super().__init__('config_server')
        self.store = {}
        self.create_service(FishBotConfig, 'fishbot_config', self.cb)

    def cb(self, req, resp):
        self.store[req.key] = req.value
        self.get_logger().info(f'set {req.key}={req.value}')
        resp.key, resp.value = req.key, self.store[req.key]
        return resp

def main():
    rclpy.init()
    rclpy.spin(ConfigServer())
    rclpy.shutdown()
```

在 `setup.py` 的 `console_scripts` 中注册 `config_server = fishbot_tools.config_server:main`，编译后：

```bash
ros2 run fishbot_tools config_server
ros2 service call /fishbot_config fishbot_interfaces/srv/FishBotConfig "{key: 'speed', value: '0.2'}"
```

## 验收点

- [ ] `rqt_graph` 能看到 turtlesim 的节点与话题连接图。
- [ ] 能用 `ros2 service call` 调通自己写的 `fishbot_config` 服务。

## 课后作业

1. 在 `fishbot_interfaces` 中新增一个 `srv/SetSpeed.srv`（请求：`float32 linear, float32 angular`；响应：`bool success, string message`），编译并用 `ros2 interface show` 截图。
2. 简答：话题、服务、动作各适合机器人中的什么场景？各举一个导航相关的例子。

## 常见问题

- `Package 'fishbot_interfaces' not found`：新终端忘记 `source install/setup.bash`。
- 同学之间话题串扰：检查 `ROS_DOMAIN_ID`。
