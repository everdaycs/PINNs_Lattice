#include "p_lattice_planner/lattice_planner.hpp"
#include <nav2_util/node_utils.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <fstream>

namespace p_lattice_planner
{

// =========================================================================================
// GridAdapter: Abstraction for Costmap interaction
// =========================================================================================
class GridAdapter
{
public:
  GridAdapter(nav2_costmap_2d::Costmap2D * costmap) : costmap_(costmap) {}

  void updateCostmap(nav2_costmap_2d::Costmap2D * costmap) { costmap_ = costmap; }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    return costmap_->worldToMap(wx, wy, mx, my);
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    costmap_->mapToWorld(mx, my, wx, wy);
  }

  unsigned char getCost(unsigned int mx, unsigned int my) const
  {
    return costmap_->getCost(mx, my);
  }

  unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return costmap_->getIndex(mx, my);
  }

  unsigned int getSizeX() const { return costmap_->getSizeInCellsX(); }
  unsigned int getSizeY() const { return costmap_->getSizeInCellsY(); }
  double getResolution() const { return costmap_->getResolution(); }

private:
  nav2_costmap_2d::Costmap2D * costmap_;
};

// =========================================================================================
// SearchCore: The A* Implementation (SE2 State Lattice)
// =========================================================================================
class SearchCore
{
public:
  static constexpr int NUM_ANGLES = 16;
  
  struct Node
  {
    int x, y;           // Grid coordinates
    int theta_idx;      // Discrete angle index [0, 15]
    uint64_t index_3d;  // Unique ID for (x, y, theta)
    double cost_g;
    double cost_h;
    double cost_f;
    uint64_t parent_index;
    double v_limit = 1.0;
    double yaw = 0.0;   // Continuous yaw

    // For priority queue
    bool operator>(const Node & other) const
    {
      if (std::abs(cost_f - other.cost_f) < 1e-4) {
          return cost_g < other.cost_g;
      }
      return cost_f > other.cost_f;
    }
  };

  struct MotionPrimitive {
    int dx, dy;
    int d_theta; 
    double length;
    double kappa; 
    double delta_yaw;
    int id; 
  };

  struct RawPrimitive {
    double l;
    double k;
    double dyaw;
    int dt_idx;
    int id;
  };

  SearchCore(GridAdapter * grid_adapter, std::shared_ptr<pinn::EdgeDynamicsEvaluator> evaluator, double h_weight = 4.0, double cost_weight = 2.0) 
    : grid_adapter_(grid_adapter), pinn_evaluator_(evaluator), h_weight_(h_weight), cost_weight_(cost_weight) 
  {
    setupPrimitives();
    precomputeRotatedPrimitives();
  }

  void setupPrimitives() {
    // 基础参数定义
    int step = 4;
    
    raw_primitives_ = {
      { (double)step, 0.0, 0.0, 0, 0 },                     // 直行
      { std::sqrt(step*step+1), 0.5, M_PI/8.0, 1, 1 },     // 左转 (R=2.0)
      { std::sqrt(step*step+1), -0.5, -M_PI/8.0, -1, 2 },  // 右转 (R=2.0)
      { std::sqrt((step-1)*(step-1)+4), 1.3333, M_PI/4.0, 2, 3 }, // 大左转 (R=0.75)
      { std::sqrt((step-1)*(step-1)+4), -1.3333, -M_PI/4.0, -2, 4 } // 大右转 (R=0.75)
    };
  }

  void precomputeRotatedPrimitives() {
    rotated_primitives_.resize(NUM_ANGLES);
    for (int t = 0; t < NUM_ANGLES; ++t) {
      double angle = t * (2.0 * M_PI / NUM_ANGLES);
      double cos_t = std::cos(angle);
      double sin_t = std::sin(angle);
      
      for (const auto& raw : raw_primitives_) {
        MotionPrimitive mp;
        double dx_rel, dy_rel;
        
        // Exact Arc Geometry
        if (std::abs(raw.k) < 1e-4) {
            dx_rel = raw.l;
            dy_rel = 0.0;
        } else {
            dx_rel = std::sin(raw.k * raw.l) / raw.k;
            dy_rel = (1.0 - std::cos(raw.k * raw.l)) / raw.k;
        }
        
        // Rotate and Round to Grid
        mp.dx = std::round(dx_rel * cos_t - dy_rel * sin_t);
        mp.dy = std::round(dx_rel * sin_t + dy_rel * cos_t);
        mp.d_theta = raw.dt_idx;
        mp.length = raw.l;
        mp.kappa = raw.k;
        mp.delta_yaw = raw.dyaw;
        mp.id = raw.id;
        rotated_primitives_[t].push_back(mp);
      }
    }
  }

  void setVehicleParams(const pinn::VehicleParams& params) { veh_ = params; }

  inline uint64_t getIndex3D(int x, int y, int theta_idx) {
    return (static_cast<uint64_t>(y) * grid_width_ + x) * NUM_ANGLES + (theta_idx < 0 ? (theta_idx + NUM_ANGLES) : (theta_idx % NUM_ANGLES));
  }

  bool search(
    const geometry_msgs::msg::Pose & start_world,
    const geometry_msgs::msg::Pose & goal_world,
    std::vector<geometry_msgs::msg::PoseStamped> & plan,
    bool allow_unknown,
    double tolerance,
    float mu,
    std::function<bool()> is_timeout)
  {
    unsigned int start_mx, start_my, goal_mx, goal_my;
    if (!grid_adapter_->worldToMap(start_world.position.x, start_world.position.y, start_mx, start_my)) {
        fprintf(stderr, "[LatticePlanner] FAILED worldToMap for Start: (%f, %f)\n", start_world.position.x, start_world.position.y);
        return false;
    }
    if (!grid_adapter_->worldToMap(goal_world.position.x, goal_world.position.y, goal_mx, goal_my)) {
        fprintf(stderr, "[LatticePlanner] FAILED worldToMap for Goal: (%f, %f)\n", goal_world.position.x, goal_world.position.y);
        return false;
    }

    grid_width_ = grid_adapter_->getSizeX();
    grid_height_ = grid_adapter_->getSizeY();
    
    fprintf(stderr, "[LatticePlanner] Search Start. Map: %dx%d, Start: %d,%d, Goal: %d,%d, Cost: %d\n", 
            grid_width_, grid_height_, start_mx, start_my, goal_mx, goal_my, (int)grid_adapter_->getCost(start_mx, start_my));

    size_t total_states = static_cast<size_t>(grid_width_) * grid_height_ * NUM_ANGLES;

    if (cost_so_far_vec_.size() != total_states) {
        cost_so_far_vec_.assign(total_states, 1e12);
        came_from_vec_.assign(total_states, 0);
    } else {
        std::fill(cost_so_far_vec_.begin(), cost_so_far_vec_.end(), 1e12);
    }

    double start_yaw = tf2::getYaw(start_world.orientation);
    int start_theta_idx = static_cast<int>(std::round(start_yaw / (2.0 * M_PI / NUM_ANGLES))) % NUM_ANGLES;
    if (start_theta_idx < 0) start_theta_idx += NUM_ANGLES;

    uint64_t start_idx_3d = getIndex3D(start_mx, start_my, start_theta_idx);

    // 计算 2D 启发式
    computeDijkstraHeuristic(goal_mx, goal_my);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    
    double h0 = heuristic(start_mx, start_my, goal_mx, goal_my) * h_weight_;
    open_list.push({(int)start_mx, (int)start_my, start_theta_idx, start_idx_3d, 0.0, h0, h0, start_idx_3d, 1.0, start_yaw});
    cost_so_far_vec_[start_idx_3d] = 0.0;
    came_from_vec_[start_idx_3d] = start_idx_3d;

    const double res = grid_adapter_->getResolution();
    const int goal_tol_cells = std::max(1, (int)(tolerance / res));

    int iterations = 0;
    while (!open_list.empty())
    {
      if (is_timeout()) return false;
      iterations++;

      Node current = open_list.top();
      open_list.pop();

      if (current.cost_g > cost_so_far_vec_[current.index_3d]) continue;

      if (iterations % 1000 == 0) {
          RCLCPP_INFO(rclcpp::get_logger("LatticePlanner"), "Iter %d, queue size %zu, current g: %.2f, h: %.2f", iterations, open_list.size(), current.cost_g, current.cost_h);
      }

      // 目标检测
      int dx_goal = current.x - (int)goal_mx;
      int dy_goal = current.y - (int)goal_my;
      if (std::abs(dx_goal) <= goal_tol_cells && std::abs(dy_goal) <= goal_tol_cells) 
      {
        reconstructPath(start_idx_3d, current.index_3d, came_from_vec_, plan);
        return true;
      }

      const auto& current_prims = rotated_primitives_[current.theta_idx];
      
      // PINN 批量准备
      std::vector<int> valid_prim_indices;
      std::vector<unsigned int> p_ids;
      std::vector<pinn::PrimitiveInfo> p_infos;
      valid_prim_indices.reserve(5);

      for (size_t i = 0; i < current_prims.size(); ++i) {
        const auto& prim = current_prims[i];
        int nx = current.x + prim.dx;
        int ny = current.y + prim.dy;

        if (nx < 0 || nx >= (int)grid_width_ || ny < 0 || ny >= (int)grid_height_) continue;

        unsigned char cost = grid_adapter_->getCost(nx, ny);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || 
            (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown)) continue;

        valid_prim_indices.push_back(i);
        if (pinn_evaluator_) {
          p_ids.push_back(prim.id);
          pinn::PrimitiveInfo info;
          info.length = prim.length * res;
          info.start_x = 0; info.start_y = 0; info.start_yaw = 0;
          info.end_x = info.length; info.end_yaw = prim.delta_yaw;
          info.max_curvature = prim.kappa;
          p_infos.push_back(info);
        }
      }

      std::vector<pinn::EvaluationResult> pinn_results;
      if (pinn_evaluator_ && !p_ids.empty()) {
        pinn_results = pinn_evaluator_->evaluateBatch(p_ids, p_infos, current.v_limit, mu, veh_);
      }

      for (size_t k = 0; k < valid_prim_indices.size(); ++k) {
        int i = valid_prim_indices[k];
        const auto& prim = current_prims[i];
        
        double v_next = current.v_limit;
        double extra_penalty = 0.0;

        if (pinn_evaluator_) {
          if (!pinn_results[k].accepted) continue;
          v_next = pinn_results[k].v_safe;
          extra_penalty = pinn_results[k].extra_cost;
        }

        int nt = (current.theta_idx + prim.d_theta + NUM_ANGLES) % NUM_ANGLES;
        int nx = current.x + prim.dx;
        int ny = current.y + prim.dy;
        uint64_t n_idx = getIndex3D(nx, ny, nt);

        unsigned char cost = grid_adapter_->getCost(nx, ny);
        double traversability_penalty = (cost / 255.0) * (prim.length * res) * cost_weight_;

        double edge_cost = (prim.length * res / (v_next + 0.1)) + std::abs(prim.d_theta) * 0.2 + extra_penalty + traversability_penalty;
        double new_g = current.cost_g + edge_cost;

        if (new_g < cost_so_far_vec_[n_idx]) {
          cost_so_far_vec_[n_idx] = new_g;
          double h = heuristic(current.x + prim.dx, current.y + prim.dy, goal_mx, goal_my) * h_weight_;
          open_list.push({current.x + prim.dx, current.y + prim.dy, nt, n_idx, new_g, h, new_g + h, current.index_3d, v_next, current.yaw + prim.delta_yaw});
          came_from_vec_[n_idx] = current.index_3d;
        }
      }
    }

    RCLCPP_WARN(rclcpp::get_logger("LatticePlanner"), "Search failed! Explored %d iterations. Open list empty: %s", iterations, open_list.empty() ? "yes" : "no");
    return false;
  }

private:
  GridAdapter * grid_adapter_;
  std::shared_ptr<pinn::EdgeDynamicsEvaluator> pinn_evaluator_;
  pinn::VehicleParams veh_;
  std::vector<std::vector<MotionPrimitive>> rotated_primitives_; // [theta_idx][primitive_idx]
  std::vector<RawPrimitive> raw_primitives_;
  double h_weight_;
  double cost_weight_;
  unsigned int grid_width_, grid_height_;

  // 预分配大数组
  std::vector<double> cost_so_far_vec_;
  std::vector<uint64_t> came_from_vec_;
  std::vector<float> dijkstra_map_; // 2D Obstacle Heuristic

  void computeDijkstraHeuristic(int goal_x, int goal_y) {
    if (dijkstra_map_.size() != (size_t)grid_width_ * grid_height_) {
        dijkstra_map_.assign((size_t)grid_width_ * grid_height_, 1e9f);
    } else {
        std::fill(dijkstra_map_.begin(), dijkstra_map_.end(), 1e9f);
    }

    struct DistNode {
        int x, y;
        float d;
        bool operator>(const DistNode& o) const { return d > o.d; }
    };
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> q;

    dijkstra_map_[goal_y * grid_width_ + goal_x] = 0;
    q.push({goal_x, goal_y, 0});

    const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float dg[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

    while (!q.empty()) { 
        DistNode curr = q.top(); q.pop();
        if (curr.d > dijkstra_map_[curr.y * grid_width_ + curr.x]) continue;

        for (int i = 0; i < 8; ++i) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];
            if (nx < 0 || nx >= (int)grid_width_ || ny < 0 || ny >= (int)grid_height_) continue;
            
            unsigned char cost = grid_adapter_->getCost(nx, ny);
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) continue;

            float new_d = curr.d + dg[i] + (float)cost / 50.0f; 
            int nidx = ny * grid_width_ + nx;
            if (new_d < dijkstra_map_[nidx]) {
                dijkstra_map_[nidx] = new_d;
                q.push({nx, ny, new_d});
            }
        }
    }
  }

  double heuristic(int x, int y, int gx, int gy)
  {
    float d2 = dijkstra_map_[y * grid_width_ + x];
    double res = grid_adapter_->getResolution();
    if (d2 > 1e7) return std::hypot(x - gx, y - gy) * res;
    return static_cast<double>(d2) * res;
  }

  void reconstructPath(
    uint64_t start_index,
    uint64_t goal_index,
    const std::vector<uint64_t> & came_from,
    std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    uint64_t current = goal_index;
    while (current != start_index)
    {
      uint64_t total_xy = current / NUM_ANGLES;
      unsigned int mx = total_xy % grid_width_;
      unsigned int my = total_xy / grid_width_;
      int theta_idx = current % NUM_ANGLES;

      double wx, wy;
      grid_adapter_->mapToWorld(mx, my, wx, wy);

      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      
      double yaw = theta_idx * (2.0 * M_PI / NUM_ANGLES);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      
      plan.push_back(pose);
      uint64_t next = came_from[current];
      if (next == current) break;
      current = next;
    }
    std::reverse(plan.begin(), plan.end());
  }
};


// =========================================================================================
// LatticePlanner Implementation
// =========================================================================================

LatticePlanner::LatticePlanner() : costmap_(nullptr)
{
}

LatticePlanner::~LatticePlanner()
{
}

void LatticePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  clock_ = node->get_clock();

  logger_ = node->get_logger();

  // Parameter declaration
  nav2_util::declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_planning_time_ms", rclcpp::ParameterValue(10000));
  nav2_util::declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".friction_mu", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, name + ".heuristic_weight", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty_weight", rclcpp::ParameterValue(10.0));

  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  node->get_parameter(name + ".max_planning_time_ms", max_planning_time_ms_);
  node->get_parameter(name + ".use_astar", use_astar_);
  node->get_parameter(name + ".friction_mu", friction_mu_);
  node->get_parameter(name + ".heuristic_weight", heuristic_weight_);
  node->get_parameter(name + ".cost_penalty_weight", cost_penalty_weight_);

  // PINN Integration
  if (parent_node_.lock()) {
    pinn_evaluator_ = std::make_shared<pinn::EdgeDynamicsEvaluator>();
    pinn_evaluator_->configure(parent_node_.lock(), name);
  }

  grid_adapter_ = std::make_unique<GridAdapter>(costmap_);
  search_core_ = std::make_unique<SearchCore>(grid_adapter_.get(), pinn_evaluator_, heuristic_weight_, cost_penalty_weight_);
}

void LatticePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up LatticePlanner...");
  grid_adapter_.reset();
  search_core_.reset();
}

void LatticePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating LatticePlanner...");
  // Need to update costmap pointer in case it changed
  if (grid_adapter_) {
      grid_adapter_->updateCostmap(costmap_ros_->getCostmap());
  }
}

void LatticePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating LatticePlanner...");
}

nav_msgs::msg::Path LatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  (void)cancel_checker;
  RCLCPP_INFO(logger_, "LatticePlanner: Planning from (%.2f, %.2f) to (%.2f, %.2f)", 
              start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  auto start_time = std::chrono::steady_clock::now();

  // 1. Lock Costmap
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // 2. Update Adapter
  grid_adapter_->updateCostmap(costmap_);

  // 3. Run Search
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  
  auto is_timeout = [&]() -> bool {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    if (elapsed > max_planning_time_ms_) {
        RCLCPP_WARN(logger_, "Planning timed out!");
        return true;
    }
    return false;
  };

  bool success = search_core_->search(
      start.pose, goal.pose, poses,
      allow_unknown_, tolerance_, friction_mu_, is_timeout);

  if (success)
  {
      path.poses = poses;
      // Add start and goal exactly
      if (!path.poses.empty()) {
          path.poses.insert(path.poses.begin(), start);
          path.poses.push_back(goal);
      }
      
      // Ensure all poses have correct header
      for (auto & p : path.poses) {
          p.header.frame_id = path.header.frame_id;
          p.header.stamp = path.header.stamp;
      }
  } else {
      RCLCPP_WARN(logger_, "LatticePlanner failed to find a path.");
  }

  return path;
}

}  // namespace p_lattice_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(p_lattice_planner::LatticePlanner, nav2_core::GlobalPlanner)
