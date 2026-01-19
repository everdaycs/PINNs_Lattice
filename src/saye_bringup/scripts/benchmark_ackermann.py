#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from ros_gz_interfaces.srv import SetEntityPose
# from ros_gz_interfaces.msg import Entity
import random
import time
import math
import subprocess
import numpy as np
import logging
import datetime
import os

# Setup logging
log_dir = "/root/colcon_ws/benchmark_logs"
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f'benchmark_failure_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
logging.basicConfig(filename=log_file, level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')

class BenchmarkRunner(Node):
    def __init__(self):
        super().__init__('benchmark_runner')
        # Using subprocess to call gz service directly as bridge support for SetEntityPose is limited
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Map handling
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.map_data = None
        self.map_info = None
        
        self.reset_metrics()

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}, Resolution: {msg.info.resolution}")

    def is_pose_valid(self, x, y, radius=0.3):
        if self.map_data is None:
            self.get_logger().warn("Map not received yet, skipping validity check.")
            return True # Assume valid if no map

        # Convert world coordinates to map indices
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        if not (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height):
            return False

        # Simple check: Is the center occupied?
        # 100 = occupied, 0 = free, -1 = unknown
        # We also want to check a small radius around the point to be safe
        check_radius_px = int(math.ceil(radius / self.map_info.resolution))
        
        x_start = max(0, mx - check_radius_px)
        x_end = min(self.map_info.width, mx + check_radius_px + 1)
        y_start = max(0, my - check_radius_px)
        y_end = min(self.map_info.height, my + check_radius_px + 1)

        submap = self.map_data[y_start:y_end, x_start:x_end]
        
        # If any cell in this region is occupied (>50) or unknown (-1), consider it invalid
        # Typically occupied is 100, free is 0. Unknown is -1.
        # We can be strict and require 0 (free).
        if np.any(submap > 50) or np.any(submap == -1):
            return False
            
        return True

    def reset_metrics(self):
        self.total_kinetic_energy = 0.0
        self.total_jerk = 0.0
        self.total_distance = 0.0
        self.last_pose = None
        self.last_linear_accel = 0.0
        self.last_vel_time = None
        self.last_vel = 0.0
        self.velocity_samples = []
        self.metric_start_time = None
        
    def odom_callback(self, msg):
        if self.metric_start_time is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate Kinetic Energy (mass assumed 1.0kg for normalized metric)
        # KE = 0.5 * m * v^2
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        self.total_kinetic_energy += 0.5 * (linear_vel ** 2)

        # Distance
        if self.last_pose is not None:
            dx = msg.pose.pose.position.x - self.last_pose.position.x
            dy = msg.pose.pose.position.y - self.last_pose.position.y
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        self.last_pose = msg.pose.pose

        self.velocity_samples.append(abs(linear_vel))

    def cmd_vel_callback(self, msg):
        # Calculate Smoothness (Jerk) 
        # Approximated by changes in acceleration commands or simply change in velocity command over time
        pass

    def set_gz_pose(self, pose):
        # Construct gz service command
        # req string format: name: "saye", position { x: ..., y: ... }, orientation { ... }
        req_str = (
            f'name: "saye", '
            f'position {{ x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z} }}, '
            f'orientation {{ x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w} }}'
        )

        cmd = [
            "gz", "service",
            "-s", "/world/my_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", req_str
        ]
        
        try:
            self.get_logger().info(f"Teleporting robot to ({pose.position.x:.2f}, {pose.position.y:.2f})...")
            result = subprocess.run(cmd, capture_output=True, text=True)
            # Check if execution was successful (gz service returns 0 usually, output contains response)
            if result.returncode == 0 and "data: true" in result.stdout:
                return True
            else:
                self.get_logger().error(f"Failed to set pose. Output: {result.stdout} Error: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Exception calling gz service: {e}")
            return False

def get_random_pose(min_x, max_x, min_y, max_y):
    pose = Pose()
    pose.position.x = random.uniform(min_x, max_x)
    pose.position.y = random.uniform(min_y, max_y)
    pose.position.z = 0.5 # Slightly above ground to prevent clipping
    
    # Random yaw
    yaw = random.uniform(-3.14, 3.14)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    pose.orientation.z = sy
    pose.orientation.w = cy
    return pose

def main():
    rclpy.init()
    
    benchmark_node = BenchmarkRunner()
    navigator = BasicNavigator()

    # Define a safe area within the map
    # Adjusted to a smaller inner area to reduce chance of spawning in obstacles
    # Was: (-8.0, 8.0, -8.0, 8.0)
    bounds = (-5.0, 5.0, -5.0, 5.0) 

    num_trials = 5
    results = []

    # Wait for Nav2 to be fully active
    # navigator.waitUntilNav2Active() 
    # Calling this might block if autostart is false, but usually it's true.

    # Wait for map
    print("Waiting for map...")
    while benchmark_node.map_data is None:
        rclpy.spin_once(benchmark_node, timeout_sec=0.1)

    for i in range(num_trials):
        print(f"\n--- Trial {i+1}/{num_trials} ---")
        
        # 1. Teleport Robot in Gazebo
        print("Resetting Robot Pose...")
        
        # Find valid start pose
        max_attempts = 100
        start_pose_msg = None
        for attempt in range(max_attempts):
            pose = get_random_pose(*bounds)
            if benchmark_node.is_pose_valid(pose.position.x, pose.position.y):
                start_pose_msg = pose
                break
        
        if start_pose_msg is None:
            print("Could not find valid start pose after multiple attempts. Skipping.")
            continue

        success = benchmark_node.set_gz_pose(start_pose_msg)
        if not success:
            print("Failed to reset pose in Gazebo. Skipping trial.")
            continue
            
        time.sleep(2.0) # Wait for dynamics to settle

        # 2. Set Initial Pose for Nav2 (AMCL)
        print("Setting Initial Pose to Nav2...")
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose = start_pose_msg
        
        # Clear costmaps to remove artifacts from old position
        navigator.clearAllCostmaps()
        navigator.setInitialPose(initial_pose)
        time.sleep(1.0) # Wait for AMCL to update
        
        # 3. Pick Goal
        # Find valid goal pose
        goal_pose_msg = None
        for attempt in range(max_attempts):
            pose = get_random_pose(*bounds)
            if benchmark_node.is_pose_valid(pose.position.x, pose.position.y):
                # Ensure goal is not too close to start
                dist = math.hypot(pose.position.x - start_pose_msg.position.x, 
                                  pose.position.y - start_pose_msg.position.y)
                if dist > 2.0: # Minimum distance 2m
                    goal_pose_msg = pose
                    break
        
        if goal_pose_msg is None:
             print("Could not find valid goal pose. Skipping.")
             continue

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose = goal_pose_msg

        print(f"Goal: ({goal_pose_msg.position.x:.2f}, {goal_pose_msg.position.y:.2f})")

        # 4. Navigate
        benchmark_node.reset_metrics()
        benchmark_node.metric_start_time = time.time()
        
        start_time = time.time()
        navigator.goToPose(goal_pose)
        
        print("Navigating...")
        while not navigator.isTaskComplete():
            rclpy.spin_once(benchmark_node, timeout_sec=0.1) # Spin to update metrics
            feedback = navigator.getFeedback()
            if feedback:
                # print(f"Distance remaining: {feedback.distance_remaining:.2f}", end='\r')
                pass
            # time.sleep(0.1) 
            
        result = navigator.getResult()
        duration = time.time() - start_time
        benchmark_node.metric_start_time = None # Stop recording
        
        success_flag = False
        status_msg = "Unknown"
        
        if result == TaskResult.SUCCEEDED:
            status_msg = "SUCCESS"
            success_flag = True
        elif result == TaskResult.CANCELED:
            status_msg = "CANCELED"
            logging.warning(f"Trial {i+1} CANCELED. Start: ({start_pose_msg.position.x:.2f}, {start_pose_msg.position.y:.2f}), Goal: ({goal_pose_msg.position.x:.2f}, {goal_pose_msg.position.y:.2f})")
        elif result == TaskResult.FAILED:
            status_msg = "FAILED"
            logging.error(f"Trial {i+1} FAILED. Duration: {duration:.2f}s")
            logging.error(f"  Start Pose: x={start_pose_msg.position.x:.2f}, y={start_pose_msg.position.y:.2f}, yaw={2*math.acos(start_pose_msg.orientation.w):.2f}")
            logging.error(f"  Goal Pose:  x={goal_pose_msg.position.x:.2f}, y={goal_pose_msg.position.y:.2f}")
            logging.error(f"  Metrics before fail: Dist={benchmark_node.total_distance:.2f}m, Energy={benchmark_node.total_kinetic_energy:.2f}J")
        
        # Calculate Average Metrics
        avg_ke = benchmark_node.total_kinetic_energy / len(benchmark_node.velocity_samples) if benchmark_node.velocity_samples else 0.0
        pose_error = 0.0 # Calc dist to goal if needed
        
        print(f"\nResult: {status_msg} | Time: {duration:.2f}s | Dist: {benchmark_node.total_distance:.2f}m | Avg KE: {avg_ke:.2f}J")
        
        results.append({
            "trial": i+1,
            "success": success_flag,
            "status": status_msg,
            "time": duration,
            "distance": benchmark_node.total_distance,
            "avg_ke": avg_ke,
            "start": (start_pose_msg.position.x, start_pose_msg.position.y),
            "goal": (goal_pose_msg.position.x, goal_pose_msg.position.y)
        })

    print("\n" + "="*60)
    print("                 BENCHMARK RESULTS                  ")
    print("="*60)
    success_count = sum(1 for r in results if r['success'])
    print(f"Total Trials: {num_trials}")
    print(f"Success Rate: {success_count}/{num_trials} ({(success_count/num_trials)*100:.1f}%)")
    
    if success_count > 0:
        avg_time = sum(r['time'] for r in results if r['success']) / success_count
        avg_dist = sum(r['distance'] for r in results if r['success']) / success_count
        avg_energy = sum(r['avg_ke'] for r in results if r['success']) / success_count
        print(f"Avg Time: {avg_time:.2f}s | Avg Dist: {avg_dist:.2f}m | Avg Energy: {avg_energy:.2f}")
    
    print("-" * 60)
    print(f"{'Trial':<6} | {'Status':<10} | {'Time(s)':<8} | {'Dist(m)':<8} | {'Energy':<8}")
    print("-" * 60)
    for r in results:
        print(f"{r['trial']:<6} | {r['status']:<10} | {r['time']:.2f}     | {r['distance']:.2f}     | {r['avg_ke']:.2f}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
