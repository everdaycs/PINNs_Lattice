# PINNs_Lattice 数据采集与处理

本项目包含两个阶段的数据采集流程，旨在为物理信息神经网络 (PINNs) 提供涵盖 **可行性 (Feasibility)** 和 **环境与动态评估 (Cost/Risk)** 的高质量训练数据。

## 数据集概览

| 阶段 | 代号 | 核心任务 | 数据量级 | 关键特征 | 标签来源 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Stage A** | `stageA` | **运动原语可行性滤波 (P1 Filter)** | 200,000 (20万) | 几何曲率 ($1/R$)、长度、初始速度 $v_0$、摩擦系数 $\mu$ | Ackermann 动力学回放 (Kinematically/Dynamically Feasible) |
| **Stage B** | `stageB` | **环境代价与风险评估 (P2 Cost)** | 50,000+ (5万起) | 几何特征 + **环境特征 (Costmap, Clearance)** + 动态特征 | 真实地图碰撞检测 + 动力学回放 |

---

## Stage A: 纯几何与动力学可行性 (Stage A)

该阶段生成的样本不依赖特定地图，通过在空白环境中枚举 Smac Planner 风格的运动原语（Motion Primitives），并在随机动态条件下进行回放。

### 1. 生成原理
1.  **几何采样 (Primitive Sampling)**: 复刻 Smac Lattice 逻辑，生成不同曲率 ($k \in [-k_{max}, k_{max}]$) 和长度 ($s$) 的圆弧轨迹。
2.  **动态采样 (Dynamic Sampling)**: 随机生成初始速度 $v_0 \in [0.5, 3.0] m/s$ 和地面摩擦系数 $\mu \in [0.2, 0.8]$。
3.  **动力学回放 (Rollout)**: 使用 **Pure Pursuit** 控制器驱动 **Kinematic Bicycle Model** 跟踪轨迹。
4.  **标签生成 (Labeling)**:
    *   **Feasible (1/0)**: 同时满足 `max_tracking_error < 0.3m` (运动学约束) 和 `max_lateral_acc < \mu \cdot g` (动力学约束)。
    *   **Risk**: 归一化综合风险分数 $Risk = \max(\frac{a_y}{\mu g}, \frac{e}{e_{thresh}}, \dots)$。

### 2. 运行生成命令
```bash
# 进入容器
docker exec -it ackermann_sim bash

# 设置环境与运行 (目标 20万样本，耗时约 10-15 分钟)
export PYTHONPATH=$PYTHONPATH:/root/colcon_ws/src
cd /root/colcon_ws
python3 scripts/data_collect/collect_stageA.py --config configs/data_collect_stageA.yaml
```

### 3. 数据集验证
```bash
python3 scripts/data_collect/verify_stageA.py data/pinn_lattice_dataset/stageA
```

---

## Stage B: 环境交互与碰撞风险 (Stage B)

该阶段模拟真实的规划过程，在复杂的栅格地图中（如 `saye_bringup/maps/map.yaml`）进行“伪搜索”，采集即时的候选边及其对应的环境信息。

### 1. 生成原理
1.  **状态采样**: 在地图自由空间随机撒点 $(x, y, \theta)$。
2.  **扩展采样**: 基于当前状态扩展出一组 Motion Primitives（候选边）。
3.  **特征提取**:
    *   `env_cost_max`: 轨迹沿途经过的最大 Costmap 代价。
    *   `env_clearance_min`: 轨迹距离最近障碍物的距离。
    *   *(Planned) env_patch*: 提取局部 Costmap 张量。
4.  **难例挖掘 (Hard Mining)**: 优先保留穿越高代价区域或贴近障碍物的样本（Risk > 0）。
5.  **标签生成**: 结合 Stage A 的动力学判定，并增加**真实地图碰撞检测**。

### 2. 运行生成命令
```bash
# 需确保容器内已加载正确地图路径 (调整 configs/data_collect_stageB.yaml 中 map.yaml_path)
export PYTHONPATH=$PYTHONPATH:/root/colcon_ws/src
python3 scripts/data_collect/collect_stageB.py --config configs/data_collect_stageB.yaml
```

---

## 数据格式说明 (Parquet)

所有数据以分片 Parquet 文件存储 (`shard_000.parquet`), 单文件约 1000 条样本。

**通用字段**:
*   `feasible` (int): 0 或 1
*   `risk` (float): 风险值
*   `v0`, `mu` (float): 动态条件
*   `max_ay`, `max_error` (float): 回放指标

**Stage A 特有**:
*   `mean_kappa`, `length`: 几何定义

**Stage B 特有**:
*   `env_cost_max`, `env_clearance_min`: 环境特征
*   `collision`: 是否发生碰撞 (int)
