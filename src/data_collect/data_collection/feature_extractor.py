import numpy as np

class FeatureExtractor:
    def __init__(self, map_manager, config):
        self.map = map_manager
        self.patch_size = 3.0 # meters
    
    def extract_env_features(self, traj):
        """
        Args:
            traj: [N, 3] path
        Returns:
            dict of features
        """
        # 1. Cost Integral
        costs = []
        clearances = []
        
        step_sample = max(1, len(traj) // 5) # Sample 5 points
        
        for i in range(0, len(traj), step_sample):
            pt = traj[i]
            x, y = pt[0], pt[1]
            
            # Simple check
            if self.map.check_collision(x, y):
                c = 254
                clr = 0.0
            else:
                c = 0
                clr = 1.0 # Placeholder
                
            costs.append(c)
            clearances.append(clr)
            
        # 2. Local Patch (Middle of trajectory)
        mid_pt = traj[len(traj)//2]
        # patch = self.map.get_patch(mid_pt[0], mid_pt[1], self.patch_size)
        # flatten_patch = patch.flatten()
        
        return {
            'env_cost_mean': np.mean(costs),
            'env_cost_max': np.max(costs),
            'env_clearance_min': np.min(clearances),
            # 'env_patch_mean': np.mean(patch)
        }
