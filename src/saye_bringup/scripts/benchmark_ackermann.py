#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from ros_gz_interfaces.srv import SetEntityPose
# from ros_gz_interfaces.msg import Entity
import random
import time
import math
import subprocess

class BenchmarkRunner(Node):
    def __init__(self):
        super().__init__('benchmark_runner')
        # Using subprocess to call gz service directly as bridge support for SetEntityPose is limited
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
    # Since we don't parse the map here, we start with a safe inner square.
    # Adjust these bounds based on your specific map (Racetrack/City).
    # Assumed safe area near origin.
    bounds = (-8.0, 8.0, -8.0, 8.0) 

    num_trials = 5
    results = []

    # Wait for Nav2 to be fully active
    # navigator.waitUntilNav2Active() 
    # Calling this might block if autostart is false, but usually it's true.

    for i in range(num_trials):
        print(f"\n--- Trial {i+1}/{num_trials} ---")
        
        # 1. Teleport Robot in Gazebo
        print("Resetting Robot Pose...")
        start_pose_msg = get_random_pose(*bounds)
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
        goal_pose_msg = get_random_pose(*bounds)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose = goal_pose_msg

        print(f"Goal: ({goal_pose_msg.position.x:.2f}, {goal_pose_msg.position.y:.2f})")

        # 4. Navigate
        start_time = time.time()
        navigator.goToPose(goal_pose)
        
        print("Navigating...")
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                # print(f"Distance remaining: {feedback.distance_remaining:.2f}", end='\r')
                pass
            time.sleep(0.1)
            
        result = navigator.getResult()
        duration = time.time() - start_time
        
        success_flag = False
        status_msg = "Unknown"
        
        if result == TaskResult.SUCCEEDED:
            status_msg = "SUCCESS"
            success_flag = True
        elif result == TaskResult.CANCELED:
            status_msg = "CANCELED"
        elif result == TaskResult.FAILED:
            status_msg = "FAILED"
        
        print(f"\nResult: {status_msg} | Time: {duration:.2f}s")
        
        results.append({
            "trial": i+1,
            "success": success_flag,
            "status": status_msg,
            "time": duration,
            "start": (start_pose_msg.position.x, start_pose_msg.position.y),
            "goal": (goal_pose_msg.position.x, goal_pose_msg.position.y)
        })

    print("\n" + "="*30)
    print("       BENCHMARK RESULTS       ")
    print("="*30)
    success_count = sum(1 for r in results if r['success'])
    print(f"Total Trials: {num_trials}")
    print(f"Success Rate: {success_count}/{num_trials} ({(success_count/num_trials)*100:.1f}%)")
    
    if success_count > 0:
        avg_time = sum(r['time'] for r in results if r['success']) / success_count
        print(f"Avg Time (Success): {avg_time:.2f}s")
    
    print("-" * 30)
    print(f"{'Trial':<6} | {'Status':<10} | {'Time (s)':<10}")
    print("-" * 30)
    for r in results:
        print(f"{r['trial']:<6} | {r['status']:<10} | {r['time']:.2f}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
