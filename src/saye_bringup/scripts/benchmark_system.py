#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose, ComputePathToPose # Add ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap 
from tf2_ros import Buffer, TransformListener 
import tf2_geometry_msgs 
# ... (保持原有 import)
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import csv
import json
import time
import math
import os
import datetime
import argparse
import threading
from collections import deque

# --- 辅助函数：角度归一化 ---
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --- 指标配置类 ---
class MetricConfig:
    def __init__(self):
        # 阈值配置
        self.pos_tolerance = 0.5        # m
        self.yaw_tolerance = 1.0        # rad (~57 deg, relaxed for Ackermann final orientation)
        self.min_turning_radius = 0.5   # m (对应 Ackermann 限制)
        self.max_curvature = 1.0 / self.min_turning_radius
        self.lateral_accel_limit = 2.0  # m/s^2
        self.planning_time_limit = 2.0  # s
        self.cte_fail_threshold = 1.0   # m
        self.inflation_radius = 0.35    # m

# --- 数据记录与分析器 ---
class DataRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 
                                                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        
        # 运行时数据容器
        self.lock = threading.Lock()
        self.reset_run_data()
        
        # 静态地图数据
        self.map_data = None
        self.map_info = None

    def reset_run_data(self):
        with self.lock:
            self.history_odom = [] # (t, x, y, yaw, v_linear, w_angular)
            self.history_cmd = []  # (t, v_cmd, w_cmd)
            self.current_plan = None # numpy array of (x,y)
            self.plan_received_time = None
            self.start_time = None
            self.replan_count = 0
            self.last_path_msg_time = 0

    def odom_cb(self, msg):
        if self.start_time is None: return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        with self.lock:
            self.history_odom.append((t, x, y, yaw, v, w))

    def cmd_cb(self, msg):
        if self.start_time is None: return
        t = time.time()
        with self.lock:
            self.history_cmd.append((t, msg.linear.x, msg.angular.z))

    def plan_cb(self, msg):
        if self.start_time is None: return
        # 这是一个新计划
        with self.lock:
            # Detect replanning (if timestamp implies new plan)
            now = time.time()
            if now - self.last_path_msg_time > 1.0: # Debounce
                self.replan_count += 1
            self.last_path_msg_time = now
            
            # Extract path
            path_points = []
            for pose in msg.poses:
                path_points.append([pose.pose.position.x, pose.pose.position.y])
            self.current_plan = np.array(path_points)
            if self.plan_received_time is None:
                self.plan_received_time = now

    def costmap_cb(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def start_recording(self):
        self.reset_run_data()
        self.start_time = time.time()
        # Initial Replan count is -1 (first plan doesn't count as replan)
        self.replan_count = -1 

    def stop_recording(self):
        end_time = time.time()
        self.replan_count = max(0, self.replan_count) # clamping
        return end_time - (self.start_time if self.start_time else end_time)

    # --- 核心指标计算 ---
    def compute_metrics(self, config: MetricConfig, goal_pose):
        with self.lock:
            odom_data = np.array(self.history_odom) # [t, x, y, yaw, v, w]
            plan_data = self.current_plan
            
        metrics = {}
        
        # 1. 基础统计
        if len(odom_data) == 0:
            return None # No Data
        
        metrics['duration'] = odom_data[-1, 0] - odom_data[0, 0]
        metrics['path_length_executed'] = np.sum(np.sqrt(np.diff(odom_data[:,1])**2 + np.diff(odom_data[:,2])**2))
        metrics['avg_speed'] = np.mean(odom_data[:, 4])
        
        # 2. 规划层指标 (针对最后一次有效 Plan)
        if plan_data is not None and len(plan_data) > 2:
            # 几何长度
            metrics['plan_length'] = np.sum(np.sqrt(np.diff(plan_data[:,0])**2 + np.diff(plan_data[:,1])**2))
            
            # 曲率分析 (Geometry-based)
            # kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^(1.5)
            dx = np.gradient(plan_data[:,0])
            dy = np.gradient(plan_data[:,1])
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            curvature = np.abs(dx * ddy - dy * ddx) / np.power(dx**2 + dy**2, 1.5)
            # Filter NaNs caused by duplicates
            curvature = np.nan_to_num(curvature)
            
            metrics['plan_max_curvature'] = np.max(curvature)
            metrics['plan_curvature_smoothness'] = np.sum(np.abs(np.diff(curvature)))
            metrics['plan_curvature_violation_rate'] = np.mean(curvature > config.max_curvature)
        else:
            # Fallback
            metrics['plan_length'] = 0.0
            metrics['plan_max_curvature'] = 0.0
        
        metrics['replan_count'] = self.replan_count

        # 3. 执行层指标
        # 侧向加速度 a_y ~= v * w
        lateral_acc = np.abs(odom_data[:, 4] * odom_data[:, 5])
        metrics['max_lateral_accel'] = np.max(lateral_acc)
        metrics['lateral_accel_violation_rate'] = np.mean(lateral_acc > config.lateral_accel_limit)
        
        # 控制平滑度 (Jerk/Accel variation)
        # v 的变化率
        if len(odom_data) > 1:
            acc_linear = np.abs(np.diff(odom_data[:, 4])) / np.diff(odom_data[:, 0])
            # Filter unstable tiny dt
            acc_linear = acc_linear[np.isfinite(acc_linear)]
            metrics['control_smoothness_v'] = np.mean(acc_linear)
        else:
            metrics['control_smoothness_v'] = 0.0

        # Cross Track Error (CTE)
        # 简化计算：对每个 Odom 点，找 Plan 中最近点的距离
        if plan_data is not None:
            ctes = []
            for i in range(0, len(odom_data), 5): # Downsample for speed
                robot_pos = odom_data[i, 1:3]
                dists = np.linalg.norm(plan_data - robot_pos, axis=1)
                ctes.append(np.min(dists))
            metrics['tracking_error_rmse'] = np.sqrt(np.mean(np.array(ctes)**2)) if ctes else 0.0
            metrics['tracking_error_max'] = np.max(ctes) if ctes else 0.0
        else:
            metrics['tracking_error_rmse'] = -1.0
            
        # 终点误差
        final_pos = odom_data[-1, 1:3]
        final_yaw = odom_data[-1, 3]
        goal_pos = np.array([goal_pose.position.x, goal_pose.position.y])
        goal_yaw = quaternion_to_yaw(goal_pose.orientation)
        
        dist_err = np.linalg.norm(final_pos - goal_pos)
        yaw_err = abs(normalize_angle(final_yaw - goal_yaw))
        
        metrics['final_dist_error'] = dist_err
        metrics['final_yaw_error'] = yaw_err
        
        success = (dist_err < config.pos_tolerance) and (yaw_err < config.yaw_tolerance)
        metrics['success'] = 1 if success else 0
        
        return metrics

# --- 主执行逻辑 ---
class BenchmarkRunner:
    def __init__(self, node: DataRecorder):
        self.node = node
        self.config = MetricConfig()
        # Actions
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Costmap services
        self.clear_global_client = self.node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_client = self.node.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()
        if self.clear_global_client.wait_for_service(timeout_sec=1.0):
            future = self.clear_global_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if self.clear_local_client.wait_for_service(timeout_sec=1.0):
            future = self.clear_local_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

    def wait_for_localization(self, target_pose, timeout=10.0):
        """Wait until TF map->saye matches the target pose within tolerance"""
        start_wait = time.time()
        print(f"Target: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f})")
        
        last_dist = 999.0
        while time.time() - start_wait < timeout:
            try:
                # Use lookup_transform with Time(0) to get the latest available transform
                # Frame name should be 'saye' as per robot description and nav2 params
                trans = self.tf_buffer.lookup_transform('map', 'saye', rclpy.time.Time())
                
                curr_x = trans.transform.translation.x
                curr_y = trans.transform.translation.y
                
                dx = curr_x - target_pose.position.x
                dy = curr_y - target_pose.position.y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if abs(dist - last_dist) > 0.05: # Only print when significant update
                    print(f"  Current Pose Estimate: ({curr_x:.2f}, {curr_y:.2f}) | Dist to target: {dist:.3f}m")
                    last_dist = dist
                    
                if dist < 0.8: # Tolerance for convergence
                    print("  Localization Converged.")
                    return True
            except Exception as e:
                # Still waiting for TF
                pass
            
            rclpy.spin_once(self.node, timeout_sec=0.2)
        
        print(f"  Localization TIMEOUT. Final Error: {last_dist:.3f}m")
        return False

    def run_case(self, case_id, start_pose, goal_pose, planner_id="GridBased"):
        print(f"\n=== Case {case_id} ===")
        print(f"Plan: {planner_id} | Start: ({start_pose.position.x:.2f}, {start_pose.position.y:.2f}) -> Goal: ({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})")
        
        # 1. Reset Robot (using GZ service via subprocess)
        self.teleport_robot(start_pose)
        time.sleep(1.0) # Wait for Gazebo physics to settle
        
        # 2. Set Initial Pose for AMCL
        print("Resetting AMCL Initial Pose...")
        self.set_amcl_pose(start_pose)
        
        # 3. Wait for Localization to converge
        print("Waiting for localization...")
        self.wait_for_localization(start_pose)
        
        # 4. Clear Costmaps
        # Do it twice with a small delay to ensure all layers are cleared after TF jump
        print("Clearing costmaps...")
        self.clear_costmaps()
        time.sleep(1.0)
        self.clear_costmaps()
        time.sleep(0.5)
        
        # 5. Invoke Navigation
        # Simplified BT XML for Jazzy: explicitly mapping ports
        bt_xml = f"""<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="NavSequence">
      <ComputePathToPose goal="{{goal}}" path="{{path}}" planner_id="{planner_id}"/>
      <FollowPath path="{{path}}" controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>"""
        bt_path = f"/tmp/nav2_bt_{planner_id}.xml"
        with open(bt_path, 'w') as f:
            f.write(bt_xml)
        
        # NOTE: If status 6 persists, it usually means the robot is in collision 
        # or the global costmap hasn't updated the robot's new position yet.
        # We add an extra small wait before sending the goal.
        time.sleep(1.0) 
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose = goal_pose
        goal_msg.behavior_tree = bt_path
        
        self.node.start_recording()
        
        print(f"Sending Goal with Planner: {planner_id} (BT: {bt_path})...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print("Goal rejected")
            return None
            
        print("Goal accepted, navigating...")
        result_future = goal_handle.get_result_async()
        
        # Spin while waiting
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Task done
        status = result_future.result().status
        duration = self.node.stop_recording()
        metrics = self.node.compute_metrics(self.config, goal_pose)
        
        if metrics:
            print(f"Result: {'SUCCESS' if metrics['success'] else 'FAIL'} (Status: {status})")
            print(f"Time: {metrics['duration']:.2f}s | CTE: {metrics['tracking_error_rmse']:.3f}m | LatAccel: {metrics['max_lateral_accel']:.2f}")
            print(f"End Error: Dist={metrics['final_dist_error']:.3f}m / Yaw={metrics['final_yaw_error']:.3f}rad")
            metrics['case_id'] = case_id
        else:
            print(f"Task Ended Instantly! Status: {status}")
            # Minimal metrics to avoid breaking CSV
            metrics = {'case_id': case_id, 'success': 0, 'duration': 0.0, 'tracking_error_rmse': -1.0}
        
        return metrics

    def teleport_robot(self, pose):
        import subprocess
        req_str = (
            f'name: "saye", '
            f'position {{ x: {pose.position.x}, y: {pose.position.y}, z: 0.1 }}, '
            f'orientation {{ x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w} }}'
        )
        cmd = [
            "gz", "service", "-s", "/world/my_world/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000", "--req", req_str
        ]
        subprocess.run(cmd, capture_output=True)

    def set_amcl_pose(self, pose):
        """Publish initial pose to AMCL to reset its estimate"""
        pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose = pose
        
        # Identity covariance
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        
        # Publish multiple times to ensure receipt
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.1)
        self.node.destroy_publisher(pub)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test_file', type=str, default='src/saye_bringup/config/test_poses.yaml', help='YAML file with test cases')
    parser.add_argument('--planners', type=str, nargs='+', default=['GridBased'], help='List of planners to benchmark (e.g. GridBased SmacPlannerHybrid)')
    parser.add_argument('--out_dir', type=str, default='benchmark_out', help='Output directory')
    args = parser.parse_args()

    rclpy.init()
    recorder = DataRecorder()
    runner = BenchmarkRunner(recorder)
    
    # Load cases
    cases = []
    # If file path is absolute, use it; else try to find it
    if os.path.exists(args.test_file):
        with open(args.test_file, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'test_cases' in data:
                cases = data['test_cases']
    
    # If no file or empty, generate random single case
    if not cases:
        print("No test file found. Using placeholder random case.")
        cases = [{'id': 0, 'start': {'x':0.0, 'y':0.0, 'yaw':0.0}, 'goal': {'x':5.0, 'y':5.0, 'yaw':0.0}}]

    # Prepare Overall Session Output
    session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    try:
        for p_id in args.planners:
            print(f"\n" + "="*50)
            print(f">>> STARTING BENCHMARK FOR PLANNER: {p_id}")
            print("="*50)
            
            res_dir = os.path.join(args.out_dir, session_timestamp, p_id)
            os.makedirs(res_dir, exist_ok=True)
            csv_file = os.path.join(res_dir, 'metrics.csv')
            
            all_metrics = []
            
            for case in cases:
                # Parse YAML Pose
                start = Pose()
                start.position.x = float(case['start']['x'])
                start.position.y = float(case['start']['y'])
                sy = math.sin(case['start']['yaw'] * 0.5)
                cy = math.cos(case['start']['yaw'] * 0.5)
                start.orientation.z = sy
                start.orientation.w = cy
                
                goal = Pose()
                goal.position.x = float(case['goal']['x'])
                goal.position.y = float(case['goal']['y'])
                sy = math.sin(case['goal']['yaw'] * 0.5)
                cy = math.cos(case['goal']['yaw'] * 0.5)
                goal.orientation.z = sy
                goal.orientation.w = cy
                
                # Run
                m = runner.run_case(case['id'], start, goal, p_id)
                
                if m:
                    all_metrics.append(m)
                    
                    # Write CSV immediately (append mode)
                    file_exists = os.path.isfile(csv_file)
                    with open(csv_file, 'a', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=m.keys())
                        if not file_exists:
                            writer.writeheader()
                        writer.writerow(m)
            
            # Summary for this planner
            if all_metrics:
                success_count = sum(m['success'] for m in all_metrics)
                avg_cte = np.mean([m['tracking_error_rmse'] for m in all_metrics])
                avg_time = np.mean([m['duration'] for m in all_metrics])
                
                summary = {
                    'planner': p_id,
                    'total_cases': len(all_metrics),
                    'success_rate': success_count / len(all_metrics),
                    'avg_tracking_error': avg_cte,
                    'avg_duration': avg_time
                }
                
                with open(os.path.join(res_dir, 'summary.json'), 'w') as f:
                    json.dump(summary, f, indent=2)
                
                print(f"\n--- {p_id} Summary ---")
                print(json.dumps(summary, indent=2))
                    
    except KeyboardInterrupt:
        print("Interrupted by user!")
        
    print(f"\nTotal session logs saved to {os.path.join(args.out_dir, session_timestamp)}")

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
