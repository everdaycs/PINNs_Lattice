import numpy as np
import math

class PrimitiveSampler:
    def __init__(self, config):
        self.config = config
        self.R_min = config['primitive']['R_min']
        self.resolution = config['primitive']['spatial_resolution']
        self.num_headings = config['primitive']['num_headings']
        
        # 预计算一些基础参数，模仿 Smac 离散化
        self.angle_quantization = 2 * np.pi / self.num_headings
        
        # 定义一些典型的 Primitive 模板 (Simplified from Smac parameters)
        # 1. Straight
        # 2. Maximum Curvature Turn (R_min)
        # 3. Intermediate Turns (Optional)
        # 我们这里通过随机采样 curvature 和 length 来生成，而不仅仅是查表，以增加数据多样性
        
    def _generate_arc(self, x0, y0, theta0, curvature, length):
        """生成圆弧或直线轨迹"""
        s_values = np.arange(0, length, self.resolution)
        if len(s_values) == 0:
            s_values = np.array([0.0])
            
        traj = np.zeros((len(s_values), 3))
        
        if abs(curvature) < 1e-4:
            # Straight
            traj[:, 0] = x0 + s_values * np.cos(theta0)
            traj[:, 1] = y0 + s_values * np.sin(theta0)
            traj[:, 2] = theta0
        else:
            # Arc
            R = 1.0 / curvature
            cx = x0 - R * np.sin(theta0)
            cy = y0 + R * np.cos(theta0)
            
            # d_theta = s / R
            # theta = theta0 + d_theta
            theta_values = theta0 + s_values * curvature
            
            traj[:, 0] = cx + R * np.sin(theta_values)
            traj[:, 1] = cy - R * np.cos(theta_values)
            traj[:, 2] = theta_values
            
        return traj

    def sample(self):
        """
        随机生成一条符合 Kinematic 约束的 Primitive
        """
        # 1. 随机起始状态 (通常假设在原点，航向对齐某个 bin，但也允许随机微扰)
        # 为了数据集多样性，我们在局部坐标系生成，start = (0,0,0)
        x0, y0, theta0 = 0.0, 0.0, 0.0
        
        # 2. 随机采样几何参数
        # 模拟 Smac: 长度通常是 resolution 的整数倍，或是 grid 的跳转
        # Length: 0.2m ~ 3.0m
        length = np.random.uniform(0.2, 3.0)
        
        # Curvature: [-1/Rmin, 1/Rmin]
        # Smac 通常是一段常曲率弧。为了更强的泛化，我们可以加一点曲率扰动吗？
        # Stage A 先只做 Constant Curvature Primitive，这是 Smac 的基础。
        max_k = 1.0 / self.R_min
        # 采样策略：增加 Straight 和 Max Turn 的概率
        r = np.random.random()
        if r < 0.2:
            k = 0.0 # Straight
        elif r < 0.4:
            k = max_k # Left Max
        elif r < 0.6:
            k = -max_k # Right Max
        else:
            k = np.random.uniform(-max_k, max_k)
            
        # 3. 生成轨迹
        traj = self._generate_arc(x0, y0, theta0, k, length)
        
        # 4. 计算特征
        # Delta x, y, theta
        dx = traj[-1, 0] - traj[0, 0]
        dy = traj[-1, 1] - traj[0, 1]
        dtheta = traj[-1, 2] - traj[0, 2]
        
        meta = {
            'length': length,
            'mean_kappa': k,
            'max_kappa': abs(k),
            'delta_x': dx,
            'delta_y': dy,
            'delta_theta': dtheta,
            'primitive_type': 'constant_curvature'
        }
        
        return traj, meta
