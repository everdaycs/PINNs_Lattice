#ifndef P_LATTICE_PLANNER__LATTICE_PLANNER_HPP_
#define P_LATTICE_PLANNER__LATTICE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace p_lattice_planner
{

// Forward declaration of internal components
class GridAdapter;
class SearchCore;

class LatticePlanner : public nav2_core::GlobalPlanner
{
public:
  LatticePlanner();
  ~LatticePlanner();

  // nav2_core::GlobalPlanner lifecycle methods
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Main planning method
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // Internal helper to check timeout
  bool isTimeout();

  // Pointer to the lifecycle node
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  
  // Clock for timing
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("LatticePlanner")};

  // Costmap handles
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Plugin name
  std::string name_;

  // Parameters
  double tolerance_;
  bool allow_unknown_;
  int max_planning_time_ms_;
  bool use_astar_;

  // Internal Components
  std::unique_ptr<GridAdapter> grid_adapter_;
  std::unique_ptr<SearchCore> search_core_;
};

}  // namespace p_lattice_planner

#endif  // P_LATTICE_PLANNER__LATTICE_PLANNER_HPP_
