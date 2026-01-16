#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from ament_index_python.packages import get_package_share_directory

import math
import os
import pickle
import glob
import time
import numpy as np
import sys

from random import seed
from random import randint
from random import uniform

# Use a fixed seed for reproducibility
seed(42)

def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = {}
    for planner in planners:
        # returns ComputePathToPose.Result if successful, None otherwise
        result = navigator._getPathImpl(initial_pose, goal_pose, planner, use_start=True)
        if result is not None and hasattr(result, 'path'):
             results[planner] = result.path
        elif result is not None:
             # Should be path if it was wrapped differently, but let's assume it's the result
             results[planner] = result
        else:
            results[planner] = None
    return results


def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res, origin_x, origin_y):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        # Simple random sampling
        w = costmap.shape[1]
        h = costmap.shape[0]
        
        # Grid coordinates
        mx = randint(side_buffer, w - side_buffer)
        my = randint(side_buffer, h - side_buffer)
        
        # Check cost
        # costmap is flattened 1D array usually or 2D if resized
        # costmap_msg.data is row-major
        # 0: free, 100: lethal, -1: unknown
        
        cost = costmap[my, mx] # numpy is (row, col) ie (y, x)
        
        if cost < max_cost and cost != -1:
             # Convert to world
             start.pose.position.x = (mx * res) + origin_x
             start.pose.position.y = (my * res) + origin_y
             start.pose.orientation.w = 1.0
             return start


def getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res, origin_x, origin_y):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        # Grid coordinates
        w = costmap.shape[1]
        h = costmap.shape[0]
        
        mx = randint(side_buffer, w - side_buffer)
        my = randint(side_buffer, h - side_buffer)
        
        cost = costmap[my, mx]
        
        if cost < max_cost and cost != -1:
             goal.pose.position.x = (mx * res) + origin_x
             goal.pose.position.y = (my * res) + origin_y
             goal.pose.orientation.w = 1.0
             
             # Check minimal distance
             dist = math.hypot(goal.pose.position.x - start.pose.position.x, 
                               goal.pose.position.y - start.pose.position.y)
             if dist > 2.0: # Minimum 2 meters
                 return goal

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Find the map in our package
    try:
        pkg_share = get_package_share_directory('p_lattice_planner')
        # We need to find where we copied the maps.
        # Since we are running from source usually, or install.
        # But wait, we copied maps to src above. We need to install them to share if we want get_package_share_directory to work
        # OR we just use a relative path if running from source root.
        
        # Let's try to find it in the current directory or via share if installed
        map_path = os.path.join(pkg_share, 'maps', '100by100_20.yaml')
        if not os.path.exists(map_path):
             # Fallback to source location assumption if not installed yet
             map_path = os.path.abspath('src/p_lattice_planner/maps/100by100_20.yaml')
             
    except Exception as e:
        print(f"Error finding map: {e}")
        return

    print(f"Loading map: {map_path}")
    navigator.changeMap(map_path)
    time.sleep(2)

    # Get the costmap for start/goal validation
    # Note: BasicNavigator updates costmap in a background thread, might need to wait
    costmap_msg = navigator.getGlobalCostmap()
    if costmap_msg is None:
        print("Failed to get costmap")
        return

    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    planners = ['GridBased', 'LatticePlanner']
    max_cost = 253 # Inscribed inflated
    side_buffer = 5
    time_stamp = navigator.get_clock().now().to_msg()
    
    results = {p: [] for p in planners}
    
    random_pairs = 10 # 10 pairs for quick test
    
    res = costmap_msg.metadata.resolution
    origin_x = costmap_msg.metadata.origin.position.x
    origin_y = costmap_msg.metadata.origin.position.y

    i = 0
    while i < random_pairs:
        print("Cycle: ", i+1, "out of: ", random_pairs)
        start = getRandomStart(costmap, max_cost, side_buffer, time_stamp, res, origin_x, origin_y)
        goal = getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res, origin_x, origin_y)
        
        print(f"  Start: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f})")
        print(f"  Goal:  ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        planner_results = getPlannerResults(navigator, start, goal, planners)
        
        for p in planners:
            path = planner_results.get(p)
            if path:
                path_len = 0.0
                # Calculate path length
                for k in range(1, len(path.poses)):
                    path_len += math.hypot(path.poses[k].pose.position.x - path.poses[k-1].pose.position.x,
                                           path.poses[k].pose.position.y - path.poses[k-1].pose.position.y)
                
                results[p].append({'success': True, 'length': path_len})
                print(f"    {p}: Success, Length: {path_len:.2f}")
            else:
                results[p].append({'success': False, 'length': 0.0})
                print(f"    {p}: Failed")
        
        i += 1

    # Simple Summary
    print("\nXXX RESULTS SUMMARY XXX")
    for p in planners:
        stats = results[p]
        success_count = sum(1 for r in stats if r['success'])
        avg_len = sum(r['length'] for r in stats if r['success']) / success_count if success_count > 0 else 0
        print(f"Planner: {p}")
        print(f"  Success Rate: {success_count}/{random_pairs} ({success_count/random_pairs*100:.1f}%)")
        print(f"  Avg Length:   {avg_len:.2f}")

if __name__ == '__main__':
    main()
