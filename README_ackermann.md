# 基于 ROS2 和 Gazebo Sim Harmonic 的阿克曼转向小车仿真

本项目展示了一个具有 **阿克曼转向 (Ackermann steering)** 能力的自定义车辆仿真，使用 **ROS2** 和 **Gazebo Sim Harmonic** 环境开发。该模型集成了多种传感器和导航工具，支持自主运行，是该仿真框架下最早实现的阿克曼转向车辆之一。

### 如果你喜欢这个项目，欢迎给它一个 ⭐ 以示支持！

## 目录

- [特性](#特性)
- [环境要求](#环境要求)
- [快速开始 (Docker)](#快速开始-docker)
- [如何启动 Benchmark](#如何启动-benchmark)
- [本地安装](#本地安装)
- [Docker 安装](#docker-安装)
- [使用指南](#使用指南)
- [未来工作](#未来工作)
- [展示](#展示)
- [TF 树](#tf-树)

## 特性

### 1. **阿克曼转向**
- 自定义车辆模型，具有真实的阿克曼转向动力学，确保精确的操纵性。

### 2. **ROS2 通信**
- 所有传感器数据和控制信号均完全集成到 ROS2 生态系统中，实现无缝互操作。

### 3. **传感器集**
- **IMU**: 提供姿态和角速度信息。
- **里程计 (Odometry)**: 确保准确的车辆状态反馈。
- **激光雷达 (LiDAR)**: 用于障碍物检测和环境扫描。
- **摄像头**:
  - 前视摄像头
  - 后视摄像头
  - 左侧摄像头
  - 右侧摄像头
  > **注意：** 默认情况下，仅前视摄像头桥接到 ROS 2。如需启用其他摄像头（左、右、后），请取消 `saye_bringup/config/ros_gz_bridge.yaml` 中相应部分的 `#` 注释（例如 `/camera/left_raw` 等）。

### 4. **导航系统**
- 集成 **Nav2 堆栈** 实现自主导航。
- 使用 **AMCL (自适应蒙特卡洛定位)** 提高位置精度。
- 实现 **SLAM** 技术进行实时建图。
- 经过精调的参数，针对阿克曼模型优化了导航性能。

### 5. **手动控制 (外接手柄)**
- 支持通过游戏手柄或键盘在仿真环境中进行手动控制，方便交互式测试。

### 6. **可视化**
- 在 **RViz2** 中完整显示模型和传感器数据，直观展示机器人状态。

## 环境要求

- **ROS2 (Jazzy / Humble)** 
- **Gazebo Sim Harmonic**
- **RViz2**
- **Nav2**

## 快速开始 (Docker)

为了方便运行（带 GUI 支持且修复了已知问题）：

1. **宿主机设置**: 允许 Docker 访问 X11 图形界面：
   ```bash
   xhost +local:root
   ```

2. **启动容器**:
   ```bash
   docker-compose up -d
   ```

3. **安装依赖并编译** (仅首次或代码修改后需要)：
   ```bash
   docker exec -it ackermann_sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/colcon_ws && colcon build --symlink-install"
   ```

4. **启动仿真**:
   ```bash
   docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && ros2 launch saye_bringup saye_spawn.launch.py"
   ```

## 如何启动 Benchmark

### 1. 启动仿真环境 (Terminal 1)
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && ros2 launch saye_bringup saye_spawn.launch.py"
```

### 2. 启动导航系统 (Terminal 2)
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && ros2 launch saye_bringup navigation_bringup.launch.py"
```

### 3. 运行 Benchmark 脚本 (Terminal 3)
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/benchmark_ackermann.py"
```

## 本地安装

1. **环境确认**: 确保安装了 Gazebo Harmonic 和对应的 `ros_gz`:
   `sudo apt-get install ros-${ROS_DISTRO}-ros-gz`

2. **克隆仓库**:
   ```bash
   mkdir -p ackermann_sim/src && cd ackermann_sim/src
   git clone https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2
   cd ..
   ```

3. **编译项目**:
   `colcon build && source install/setup.bash`

4. **设置环境变量**:
   ```bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/ackermann-vehicle-gzsim-ros2/
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/src/ackermann-vehicle-gzsim-ros2/
   ```

## 使用指南

### 1. 基础仿真与手动控制
- 启动仿真: `ros2 launch saye_bringup saye_spawn.launch.py`
- 键盘控制: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### 2. SLAM 建图
- 启动仿真后，运行 SLAM Toolbox：
  `ros2 launch saye_bringup slam.launch.py`

### 3. Nav2 自主导航
- 启动仿真后，运行导航堆栈：
  `ros2 launch saye_bringup navigation_bringup.launch.py`

## 未来工作
1. **3D SLAM 支持**: 使用先进的 DRL 算法训练车辆自主处理复杂场景。
2. **增强特性**: 探索更多传感器配置和导航策略。
3. **3D 定位集成**: 引入比 AMCL (2D) 更精确鲁棒的定位算法。

## 展示

![Simulation Image](https://github.com/user-attachments/assets/dd5604c6-014e-4a7a-9a2f-c4dd237abb37)

| **Gazebo Sim Harmonic** | **RViz2** |
| :---: | :---: |
| ![GzSim](https://github.com/user-attachments/assets/1d2b56f7-34c1-4b01-9a85-fb01ceab5bd6) | ![RViz](https://github.com/user-attachments/assets/ba6853fd-4143-4b4d-bbc6-072895e4c75e) |

## TF 树
![TF Tree](saye_msgs/readme_files/frames.png)

---

## 注意事项
- **Robot Reset**: Benchmark 脚本通过 `gz service` 重置位置，绕过了 ROS 桥接的限制。
- **图形界面**: 如果看到 `X11 connection failed`，请确保在宿主机执行了 `xhost +`。
