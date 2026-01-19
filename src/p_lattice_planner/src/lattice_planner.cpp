#include "p_lattice_planner/lattice_planner.hpp"
#include <nav2_util/node_utils.hpp>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

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

private:
  nav2_costmap_2d::Costmap2D * costmap_;
};

// =========================================================================================
// SearchCore: The A* Implementation
// =========================================================================================
class SearchCore
{
public:
  struct Node
  {
    unsigned int x, y;
    unsigned int index;
    double cost_g;
    double cost_h;
    double cost_f;
    unsigned int parent_index; // To reconstruct path

    // For priority queue
    bool operator>(const Node & other) const
    {
      return cost_f > other.cost_f;
    }
  };

  SearchCore(GridAdapter * grid_adapter) : grid_adapter_(grid_adapter) {}

  // A* Search
  bool search(
    const geometry_msgs::msg::Point & start_world,
    const geometry_msgs::msg::Point & goal_world,
    std::vector<geometry_msgs::msg::PoseStamped> & plan,
    bool allow_unknown,
    double tolerance,
    std::function<bool()> is_timeout)
  {
    (void)tolerance; // Not using tolerance for simplified grid search for now

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!grid_adapter_->worldToMap(start_world.x, start_world.y, start_x, start_y) ||
        !grid_adapter_->worldToMap(goal_world.x, goal_world.y, goal_x, goal_y))
    {
      return false; // Out of bounds
    }

    auto start_index = grid_adapter_->getIndex(start_x, start_y);
    auto goal_index = grid_adapter_->getIndex(goal_x, goal_y);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::unordered_map<unsigned int, double> cost_so_far; // Index -> G cost
    std::unordered_map<unsigned int, unsigned int> came_from; // Index -> Parent Index

    open_list.push({start_x, start_y, start_index, 0.0, heuristic(start_x, start_y, goal_x, goal_y), 0.0, start_index});
    cost_so_far[start_index] = 0.0;
    came_from[start_index] = start_index;

    const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 1, 0, -1, 1, -1, -1, 1};
    const double move_cost[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!open_list.empty())
    {
      if (is_timeout()) return false;

      Node current = open_list.top();
      open_list.pop();

      if (current.index == goal_index)
      {
        reconstructPath(start_index, goal_index, came_from, plan);
        return true;
      }

      // 8-connected grid
      for (int i = 0; i < 8; ++i)
      {
        unsigned int next_x = current.x + dx[i];
        unsigned int next_y = current.y + dy[i];

        // Bounds check
        if (next_x >= grid_adapter_->getSizeX() || next_y >= grid_adapter_->getSizeY()) continue;

        unsigned int next_index = grid_adapter_->getIndex(next_x, next_y);
        unsigned char cost = grid_adapter_->getCost(next_x, next_y);

        // Collision check
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
            cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || 
            cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown)
        {
          continue;
        }

        double new_cost = cost_so_far[current.index] + move_cost[i];
        
        // Add cost penalty for traversing high cost areas (optional, basic A* just uses distance usually)
        if (cost > 0 && cost != 255) {
            new_cost += cost / 255.0; // Minimal penalty
        }

        if (cost_so_far.find(next_index) == cost_so_far.end() || new_cost < cost_so_far[next_index])
        {
          cost_so_far[next_index] = new_cost;
          double h = heuristic(next_x, next_y, goal_x, goal_y);
          open_list.push({next_x, next_y, next_index, new_cost, h, new_cost + h, current.index});
          came_from[next_index] = current.index;
        }
      }
    }

    return false; // No path found
  }

private:
  GridAdapter * grid_adapter_;

  double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
  {
    // Euclidean distance
    double dx = static_cast<double>(x1) - static_cast<double>(x2);
    double dy = static_cast<double>(y1) - static_cast<double>(y2);
    return std::sqrt(dx * dx + dy * dy);
  }

  void reconstructPath(
    unsigned int start_index,
    unsigned int goal_index,
    const std::unordered_map<unsigned int, unsigned int> & came_from,
    std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    unsigned int current = goal_index;
    while (current != start_index)
    {
      unsigned int mx, my;
      // Reverse calculation of index not stored in GridAdapter easily without size
      // We know index = my * size_x + mx -> mx = index % size_x, my = index / size_x
      mx = current % grid_adapter_->getSizeX();
      my = current / grid_adapter_->getSizeX();

      double wx, wy;
      grid_adapter_->mapToWorld(mx, my, wx, wy);

      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);

      current = came_from.at(current);
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
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_planning_time_ms", rclcpp::ParameterValue(2000));
  nav2_util::declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(true));

  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  node->get_parameter(name + ".max_planning_time_ms", max_planning_time_ms_);
  node->get_parameter(name + ".use_astar", use_astar_);

  grid_adapter_ = std::make_unique<GridAdapter>(costmap_);
  search_core_ = std::make_unique<SearchCore>(grid_adapter_.get());
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
      start.pose.position, goal.pose.position, poses,
      allow_unknown_, tolerance_, is_timeout);

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
