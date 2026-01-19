# PINNs_Lattice

用于晶格结构（Lattice structures）的物理信息神经网络（PINNs）。

## 快速开始 (阿克曼小车仿真)

本项目包含一个在 Docker 容器中运行的 ROS2 Jazzy + Gazebo Harmonic 仿真环境。

### 0. 准备工作 (宿主机)
在启动之前，请在宿主机终端执行以下命令以允许 Docker 访问图形界面：
```bash
xhost +
```

### 1. 启动仿真环境 (终端 1)
启动 Gazebo 世界并加载车辆模型：
```bash
# 进入容器
sudo docker exec -it ackermann_sim bash

# 在容器内执行：
source /opt/ros/jazzy/setup.bash
cd /root/colcon_ws
source install/setup.bash

# 启动仿真
ros2 launch saye_bringup saye_spawn.launch.py
```
**注意**：如果仿真处于暂停状态，请在 Gazebo 界面点击下方的 **Play (▶)** 按钮。

### 2. 启动导航系统 (终端 2)
启动 Nav2 堆栈和 AMCL 定位：
```bash
# 进入容器
sudo docker exec -it ackermann_sim bash

# 在容器内执行：
source /opt/ros/jazzy/setup.bash
cd /root/colcon_ws
source install/setup.bash
ros2 launch saye_bringup navigation_bringup.launch.py
```

### 3. 运行 Benchmark 测试脚本 (终端 3)
运行自动化测试脚本，该脚本会自动重置机器人位置并设置导航目标：
```bash
# 进入容器
sudo docker exec -it ackermann_sim bash

# 在容器内执行：
source /opt/ros/jazzy/setup.bash
cd /root/colcon_ws
source install/setup.bash
python3 src/saye_bringup/scripts/benchmark_ackermann.py
```

## 注意事项
- **机器人重置**：`benchmark_ackermann.py` 脚本使用 `gz service` 命令直接重置机器人位姿，绕过了 ROS 桥接的限制。
- **重新编译**：如果您修改了源码，请在容器内执行 `colcon build --symlink-install` 以更新安装目录。
- **显示问题**：如果看到 `X11 connection failed` 错误，请确保在宿主机执行了 `xhost +`。
