# PINNs_Lattice

本项目结合了**物理信息神经网络 (PINNs)** 与**阿克曼转向小车仿真**，旨在研究晶格结构中的路径规划与控制。

---

## 项目结构 (Integrated Workspace)

项目已整合为一个统一的 ROS 2 工作空间，结构如下：

```text
PINNs_Lattice/
├── src/
│   ├── saye_bringup/       # 仿真启动、Rviz 配置与 Benchmark 脚本
│   ├── saye_description/   # 机器人 SDF 模型、传感器配置与 Gazebo 世界
│   ├── saye_control/       # 车辆控制接口与服务
│   ├── saye_msgs/          # 自定义消息与服务定义
│   └── p_lattice_planner/  # 基于神经网络的 Nav2 全局规划器插件
├── Dockerfile              # 包含 ROS 2 Jazzy + Gazebo Harmonic 的环境定义
└── docker-compose.yaml     # 定义显卡加速、挂载路径与环境变量
```

---

## 快速开始

### 1. 环境准备 (宿主机)

1. **图形界面授权** (每次重启宿主机后需执行一次)：
   ```bash
   xhost +
   ```

2. **启动 Docker 容器**：
   ```bash
   docker-compose up -d
   ```

---

### 2. 编译项目 (容器内)

在初次启动或修改 `src` 下的代码后，需要重新编译：
```bash
docker exec -it ackermann_sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/colcon_ws && colcon build --symlink-install"
```

---

### 3. 系统运行流程

请按顺序在三个终端中执行：

#### **终端 1：启动仿真世界**
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && ros2 launch saye_bringup saye_spawn.launch.py"
```
*注意：若 Gazebo 界面未运行，请点击界面底部的 **Play (▶)** 按钮。*

#### **终端 2：启动导航堆栈 (Nav2)**
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && ros2 launch saye_bringup navigation_bringup.launch.py"
```

#### **终端 3：运行 Benchmark 自动化评测**
运行此脚本将自动重置小车位置，并根据预设坐标进行导航性能分析。支持多规划器顺序对比：

**1. 基础运行 (支持多个规划器顺序测试)**：
```bash
docker exec -it ackermann_sim bash -c "source /root/colcon_ws/install/setup.bash && python3 /root/colcon_ws/src/saye_bringup/scripts/benchmark_system.py --planners GridBased SmacPlannerHybrid SmacPlannerLattice LatticePlanner --out_dir /root/colcon_ws/benchmark_results --test_file /root/colcon_ws/src/saye_bringup/config/test_poses.yaml"
```

**2. 各规划器说明与单独测试命令**：
*   **GridBased (Dijkstra)**: 提供理论最短路径基准。
    ```bash
    --planners GridBased
    ```
*   **SmacPlannerHybrid (Hybrid A*)**: 官方阿克曼连续空间搜索基准。
    ```bash
    --planners SmacPlannerHybrid
    ```
*   **SmacPlannerLattice (State Lattice)**: 官方预计算运动基元基准。
    ```bash
    --planners SmacPlannerLattice
    ```
*   **LatticePlanner (PINNs)**: 本项目研究的基于神经网络的规划器。
    ```bash
    --planners LatticePlanner
    ```

**参数说明：**
* `--planners`: 规划器列表，支持同时传入多个，程序将按顺序全自动跑完所有测试用例。
* `--test_file`: 测试用例配置文件路径 (默认为 `src/saye_bringup/config/test_poses.yaml`)。
* `--out_dir`: 结果输出目录，每个规划器将拥有独立的子文件夹。

**结果查看：**
运行结束后，可在宿主机查看 `PINNs_Lattice/benchmark_results/<时间戳>/<规划器名>` 目录下的 `metrics.csv` 和 `summary.json`。

---

## 注意事项


*   **重置机制**：Benchmark 脚本通过直接调用 `gz service` API 实现瞬移重置，不依赖 ROS Topic，因此响应速度更快。
*   **网络设置**：容器内部与宿主机共享网络，若需查看传感器原始数据，可在宿主机安装 ROS 2 Jazzy 并直接订阅相关话题。

