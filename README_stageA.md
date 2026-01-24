# Stage A: Motion Primitive Feasibility Dataset

该数据集用于训练 PINN 中的 Feasibility Filter (P1)。它包含两类信息：
1. **几何特征 (X_geom)**: 来自 Smac Lattice 生成逻辑的 Primitive 片段（Length, Curvature, Endpoint）。
2. **动态特征 (X_dyn)**: 初始速度 $v_0$、摩擦系数 $\mu$。
3. **标签 (Y)**: 在 Ackermann 动力学模型下回放该 Primitive 的可行性结果（feasible, max_ay, max_error 等）。

## 快速开始

### 1. 生成数据
在 Docker 容器中执行：

```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/src
python3 scripts/data_collect/collect_stageA.py --config configs/data_collect_stageA.yaml
```

输出将保存在 `data/pinn_lattice_dataset/stageA/`，格式为 Parquet。

### 2. 配置说明 (`configs/data_collect_stageA.yaml`)

```yaml
num_samples: 200000          # 总样本数
batch_size: 1000             # 每个 Parquet 文件的样本数
num_workers: 8               # 并行进程数

vehicle:
  wheelbase: 0.2255          # saye 小车轴距
  max_steering_angle: 0.40   # 最大转角
  # ...

primitive:
  R_min: 0.533               # 物理最小转弯半径
  spatial_resolution: 0.05   # 轨迹点间距
```

## 数据字段详解

| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| **primitive_type** | string | 'constant_curvature' 等 |
| **mean_kappa** | float | 路径平均曲率 ($1/R$) |
| **length** | float | 路径弧长 (m) |
| **v0** | float | 初始速度 (m/s) |
| **mu** | float | 地面摩擦系数 (0.1~1.0) |
| **feasible** | int | 1=可行, 0=不可行 (违反动力学或运动学约束) |
| **risk** | float | 风险归一化分数 (>1 表示不可行) |
| **max_ay** | float | 最大横向加速度 (m/s^2) |
| **max_error** | float | 最大跟踪误差 (m) (Pure Pursuit 跟踪结果) |

## 扩展指南

若需加入地图障碍物特征（Stage B）：
1. 修改 `PrimitiveSampler` 以在局部 Costmap Patch 中采样。
2. 修改 `RolloutSim` 加入碰撞检测 (`check_collision(pose, map)`).
3. 在 `collect_stageA.py` 中增加 Map 生成逻辑。
