import numpy as np
import math

class RolloutSim:
    def __init__(self, config):
        self.config = config
        self.L = config['vehicle']['wheelbase']
        self.delta_max = config['vehicle']['max_steering_angle']
        self.dt = config['rollout']['dt']
        self.ld_gain = 0.5 # Lookahead = ld_gain * v + ld_min
        self.ld_min = config['rollout']['lookahead_dist']
        
    def _normalize_angle(self, angle):
        while angle > np.pi: angle -= 2.0 * np.pi
        while angle < -np.pi: angle += 2.0 * np.pi
        return angle

    def _get_target_point(self, curr_pos, traj, lookahead_dist):
        """Pure Pursuit: Find point on traj at distance lookahead_dist from curr_pos"""
        # Simple search: find closest point first, then search forward
        dists = np.linalg.norm(traj[:, :2] - curr_pos[:2], axis=1)
        min_idx = np.argmin(dists)
        
        # Search forward for intersection
        for i in range(min_idx, len(traj)):
            d = np.linalg.norm(traj[i, :2] - curr_pos[:2])
            if d > lookahead_dist:
                return traj[i], i
                
        return traj[-1], len(traj)-1

    def run_rollout(self, traj, v0, mu, map_manager=None):
        # State: [x, y, theta]
        # Traj: [N, 3]
        
        # Init state (perfectly aligned at start)
        x, y, theta = traj[0]
        
        # Logs
        lateral_accs = []
        tracking_errors = []
        steers = []
        
        sim_time = 0.0
        max_time = len(traj) * self.config['primitive']['spatial_resolution'] / max(v0, 0.1) * 2.0 # Timeout guard
        
        completed = False
        collision = False 
        
        # Simple dynamics loop
        idx_on_traj = 0
        
        g = 9.81
        v = v0
        
        step_limit = int(max_time / self.dt)
        
        for step in range(step_limit):
            # 0. Map Collision Check
            if map_manager and map_manager.check_collision(x, y):
                collision = True
                break

            # 1. Calculate Tracking Error (Cross Track Error to closest point)
            dists = np.linalg.norm(traj[:, :2] - np.array([x,y]), axis=1)
            min_dist = np.min(dists)
            closest_idx = np.argmin(dists)
            
            # Signed error? Just magnitude for feasibility check
            tracking_errors.append(min_dist)
            
            # Check end condition
            if np.linalg.norm(traj[-1, :2] - np.array([x,y])) < 0.2:
                completed = True
                break
                
            # 2. Pure Pursuit Control
            ld = self.ld_min # + self.ld_gain * v
            target_pt, t_idx = self._get_target_point(np.array([x,y,theta]), traj, ld)
            
            alpha = self._normalize_angle(np.arctan2(target_pt[1] - y, target_pt[0] - x) - theta)
            delta = np.arctan2(2.0 * self.L * np.sin(alpha), ld)
            
            # Clip Steering
            delta = np.clip(delta, -self.delta_max, self.delta_max)
            steers.append(delta)
            
            # 3. Dynamics Update (Kinematic Bicycle)
            # x_dot = v * cos(theta)
            # y_dot = v * sin(theta)
            # theta_dot = v / L * tan(delta)
            
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += v / self.L * np.tan(delta) * self.dt
            theta = self._normalize_angle(theta)
            
            # 4. Lat Accel Check
            # a_lat = v * w = v * (v/L * tan(delta)) = v^2/L * tan(delta)
            # For small angles tan(delta) ~ delta. 
            # Ideally calculate based on temporal change of velocity vector, but this is standard approx for kinematic
            curvature = np.tan(delta) / self.L
            a_lat = v**2 * abs(curvature)
            lateral_accs.append(a_lat)
            
        
        # Aggregation
        max_ay = np.max(lateral_accs) if lateral_accs else 0.0
        max_err = np.max(tracking_errors) if tracking_errors else 0.0
        max_delta = np.max(np.abs(steers)) if steers else 0.0
        
        # Feasibility Rules
        # feasible = 1 当且仅当：max_ay <= mu*g 且 max|delta| <= delta_max 且 max_error <= e_thresh
        # Note: delta was already clipped, so if pure pursuit wanted > delta_max, the error would increase.
        # So we check tracking error as the proxy for kinematic feasibility.
        
        e_thresh = 0.3 # 30cm error allowed?
        
        is_kinematically_feasible = (max_err < e_thresh) # If we clipped steering and still tracked well, it was feasible.
        is_dynamically_feasible = (max_ay <= mu * g)
        
        feasible = is_kinematically_feasible and is_dynamically_feasible and completed and (not collision)
        
        # Risk score (0~1, >1 is bad)
        risk_ay = max_ay / (mu * g) if mu > 0 else 999
        risk_steer = max_delta / self.delta_max
        risk_err = max_err / e_thresh
        risk = max(risk_ay, risk_steer, risk_err)
        if collision:
            risk = 2.0 # High penalty for collision
        
        return {
            'feasible': int(feasible),
            'risk': risk,
            'max_ay': max_ay,
            'max_delta': max_delta,
            'max_error': max_err,
            'rollout_success': int(completed),
            'collision': int(collision)
        }
