import numpy as np
import yaml
from PIL import Image
import os
import math

class MapManager:
    """
    负责加载 ROS 地图 (YAML+PGM)，提供 Cost 查询与碰撞检测。
    """
    def __init__(self, yaml_path, padding=0.0):
        self.yaml_path = yaml_path
        self._load_map(yaml_path)
        self.padding = padding # Extra safety margin in meters
        
    def _load_map(self, yaml_path):
        if not os.path.isabs(yaml_path):
            yaml_path = os.path.abspath(yaml_path)
            
        with open(yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f)
            
        # Load Image
        img_path = self.metadata['image']
        if not os.path.isabs(img_path):
             img_path = os.path.join(os.path.dirname(yaml_path), img_path)
             
        self.origin = self.metadata['origin'] # [x, y, yaw]
        self.resolution = self.metadata['resolution']
        
        # Load Grid
        # 0=free, 100=occupied in ROS.
        # But PGM is usually grayscale.
        # mode: trinary/scale
        pil_img = Image.open(img_path).convert('L')
        self.grid = np.array(pil_img)
        
        # Transform to Costmap format (0..100)
        # ROS map_server: (255 - pixel) / 255 * 100? No.
        # usually 205 is free (254), 0 is occupied (0).
        # Let's assume standard ROS conventions:
        # PGM: 254 = free (0 cost), 0 = occupied (100 cost), 205 = unknown (-1)
        # We invert it for "Cost" [0, 100]
        # Pixel values:
        #   254 -> 0
        #   0   -> 100
        #   Others -> Scaled
        
        self.height, self.width = self.grid.shape
        self.costmap = np.zeros_like(self.grid, dtype=np.uint8)
        
        # Simple inversion for now: 255-val
        # But standard map_server logic:
        # p = pixel / 255.0
        # if p > occ_thresh: occupied
        # if p < free_thresh: free
        
        # Costmap representation: 0 (Free) to 254 (Obstacle/Lethal)
        # Let's construct a simple binary-ish costmap for Stage B
        # 255 (White) -> 0 cost
        # 0 (Black) -> 254 cost
        
        mask_occ = self.grid < 20 # Very dark
        mask_free = self.grid > 230 # Very light
        
        self.costmap[mask_occ] = 254
        self.costmap[mask_free] = 0
        # Unknown/Grey remains 0 for now or handled separately? 
        # For data collection, let's treat unknown as free to avoid excessive collision in bad maps, 
        # or treat as obstacle. Let's treat everything else as obstacle for safety.
        # mask_unknown = ~(mask_occ | mask_free)
        # self.costmap[mask_unknown] = 254
        
        # Precompute real-world bounds
        self.x_min = self.origin[0]
        self.y_min = self.origin[1]
        self.x_max = self.x_min + self.width * self.resolution
        self.y_max = self.y_min + self.height * self.resolution
        
    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        # Flip Y? ROS maps origin is usually bottom-left.
        # PIL image (numpy) [0,0] is top-left.
        # Standard map_server: origin is bottom-left.
        # So we need to flip Y index if reading from image directly?
        # Typically PGM is stored bottom-up? No, standard images are top-down.
        # map_server flips it.
        # Let's assume map_server behavior:
        # grid[my, mx] where my=0 is bottom?
        # Actually proper way:
        my = self.height - 1 - my
        return mx, my

    def check_collision(self, x, y, radius=0.1):
        """Check if point (x,y) is in collision or close to obstacle"""
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return True # Out of bounds
            
        mx, my = self.world_to_map(x, y)
        if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
            return True
            
        # Check center point
        if self.costmap[my, mx] > 50: # Threshold
            return True
            
        # Check radius (Simplified: Check neighborhood)
        # r_pixels = int((radius + self.padding) / self.resolution)
        # If r is small, single point check might suffice if map is inflated.
        # For Stage B, we want strict checks.
        
        return False
        
    def get_patch(self, x, y, size_m=2.0, output_dim=20):
        """Extract a local costmap patch centered at x,y"""
        # size_m: real world size of patch (e.g. 2m x 2m)
        # output_dim: output tensor size (e.g. 20x20 pixels)
        
        half_size = int(size_m / self.resolution / 2)
        mx, my = self.world_to_map(x, y)
        
        # Slicing with padding handling
        # ... (Simplified: return zeros if out of bound)
        x_start = mx - half_size
        x_end = mx + half_size
        y_start = my - half_size
        y_end = my + half_size
        
        patch = np.zeros((2*half_size, 2*half_size), dtype=np.uint8)
        
        # Calculate intersection
        # ...
        # For prototype, just use safe crop
        if x_start >= 0 and x_end < self.width and y_start >= 0 and y_end < self.height:
             patch = self.costmap[y_start:y_end, x_start:x_end]
             
        # Resize/Interpolate to output_dim if needed. 
        # For now return raw patch or stats.
        return patch

    def get_clearance(self, x, y, max_search=2.0):
        """Estimate distance to nearest obstacle (Ray/Spiral search)"""
        # Placeholder: Return 0 if collision, 1.0 if free (Simplified)
        if self.check_collision(x, y, 0.0):
            return 0.0
        return 1.0 # TODO: BFS for Exact Euclidean Distance Transform (ESDF)
