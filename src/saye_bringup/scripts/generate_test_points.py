#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
import os
import random
import math
import sys

class PointGenerator(Node):
    def __init__(self):
        super().__init__('point_generator')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.map_data = None
        self.map_info = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.get_logger().info("Map received.")
        
        # Once map is received, logic runs and then we can exit
        self.generate_points(30) # Generate 30 pairs
        sys.exit(0)

    def is_free(self, mx, my, radius_px):
        if not (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height):
            return False
        
        x_start = max(0, mx - radius_px)
        x_end = min(self.map_info.width, mx + radius_px + 1)
        y_start = max(0, my - radius_px)
        y_end = min(self.map_info.height, my + radius_px + 1)
        
        submap = self.map_data[y_start:y_end, x_start:x_end]
        return not (np.any(submap > 50) or np.any(submap == -1))

    def generate_points(self, count):
        points = []
        attempts = 0
        radius = 0.4 # m
        resolution = self.map_info.resolution
        radius_px = int(math.ceil(radius / resolution))
        
        width = self.map_info.width
        height = self.map_info.height
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        self.get_logger().info(f"Generating {count} pairs...")
        
        pairs = []
        while len(pairs) < count and attempts < 10000:
            attempts += 1
            
            # Random Pixel Coords
            mx1, my1 = random.randint(0, width-1), random.randint(0, height-1)
            
            if not self.is_free(mx1, my1, radius_px):
                continue
                
            mx2, my2 = random.randint(0, width-1), random.randint(0, height-1)
             
            if not self.is_free(mx2, my2, radius_px):
                continue

            # World Coords
            wx1 = origin_x + (mx1 + 0.5) * resolution
            wy1 = origin_y + (my1 + 0.5) * resolution
            wx2 = origin_x + (mx2 + 0.5) * resolution
            wy2 = origin_y + (my2 + 0.5) * resolution
            
            # Distance check
            dist = math.hypot(wx1-wx2, wy1-wy2)
            if dist < 3.0 or dist > 20.0:
                continue
                
            pairs.append({
                'id': len(pairs),
                'start': {'x': float(wx1), 'y': float(wy1), 'yaw': random.uniform(-3.14, 3.14)},
                'goal': {'x': float(wx2), 'y': float(wy2), 'yaw': random.uniform(-3.14, 3.14)} 
            })
            
        if not pairs:
            self.get_logger().error("No valid pairs found.")
            return

        data = {'test_cases': pairs}
        
        # Save to source path if possible, assuming Docker workspace structure
        output_file = "/root/colcon_ws/src/saye_bringup/config/test_poses.yaml"
        
        try:
           # Ensure directory exists
           os.makedirs(os.path.dirname(output_file), exist_ok=True)
           with open(output_file, 'w') as f:
               yaml.dump(data, f)
           self.get_logger().info(f"Saved {len(pairs)} cases to {output_file}")
        except Exception as e:
           self.get_logger().error(f"Failed to write to {output_file}: {e}")
           # Fallback
           output_file = "test_poses.yaml"
           with open(output_file, 'w') as f:
               yaml.dump(data, f)
           self.get_logger().info(f"Saved to {os.path.abspath(output_file)}")

def main():
    rclpy.init()
    node = PointGenerator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
