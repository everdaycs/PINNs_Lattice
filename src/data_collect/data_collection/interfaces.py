
class PrimitiveSampler:
    """
    负责生成符合 Smac Lattice 逻辑的几何运动元 (Primitives)。
    不依赖 ROS 消息，仅依赖 numpy。
    """
    def __init__(self, config: dict):
        pass
        
    def sample_primitive(self):
        """
        随机生成一条 Primitive
        Returns:
            traj_points: np.ndarray [N, 3] (x, y, yaw) 局部坐标系
            meta: dict (curvature, length, type, etc.)
        """
        pass

class RolloutSim:
    """
    负责在 Ackermann 动力学模型下回放 Primitive。
    包含简单的 Pure Pursuit 跟踪器。
    """
    def __init__(self, vehicle_params: dict):
        pass
        
    def run_rollout(self, traj_points, v0, mu):
        """
        执行动力学回放
        Args:
            traj_points: 参考轨迹
            v0: 初始速度
            mu: 摩擦系数
        Returns:
            result_dict: {
                'feasible': bool,
                'max_tracking_error': float,
                'max_lat_acc': float,
                'max_steer': float,
                'success': bool
            }
        """
        pass

class DatasetWriter:
    """
    负责将样本写入 Parquet 分片。
    """
    def write_batch(self, batch_data: list):
        pass
