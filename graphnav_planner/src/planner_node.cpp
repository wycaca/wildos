#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <optional>
#include <stdexcept>
#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode(const rclcpp::NodeOptions& options)
    : Node("planner_node", options)
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_, this)
    , planner_(this->get_logger())
  {
    this->declare_parameter("frontier_dist_cost_factor", 2.0);
    this->declare_parameter("goal_dist_cost_factor", 1.0);
    this->declare_parameter("frontier_score_factor", 20.0);
    this->declare_parameter("frontier_continuity_radius", 5.0);
    this->declare_parameter("frontier_progress_timeout", 12.0);
    this->declare_parameter("revisit_cost_factor", 1.0);

    const auto nonnegative_parameter = [this](const std::string& name) {
      const double value = this->get_parameter(name).as_double();
      if (value < 0.0)
      {
        throw std::invalid_argument(name + " must be nonnegative");
      }
      return value;
    };
    planner_.frontier_dist_cost_factor_ = nonnegative_parameter("frontier_dist_cost_factor");
    planner_.goal_dist_cost_factor_ = nonnegative_parameter("goal_dist_cost_factor");
    planner_.frontier_score_factor_ = nonnegative_parameter("frontier_score_factor");
    planner_.frontier_continuity_radius_ = nonnegative_parameter("frontier_continuity_radius");
    planner_.frontier_progress_timeout_ = nonnegative_parameter("frontier_progress_timeout");
    planner_.revisit_cost_factor_ = nonnegative_parameter("revisit_cost_factor");

    planner_.set_trav_class("default");

    this->declare_parameter("goal_radius", 3.0);
    goal_radius_ = this->get_parameter("goal_radius").as_double();

    graph_sub_ = this->create_subscription<graphnav_msgs::msg::NavigationGraph>(
        "~/nav_graph", 10, [this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg) {
          this->planner_.update_graph(msg);
          this->latest_graph_header_ = msg->header;
          this->plan_to_goal();
        });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          if (this->goal_pose_ && same_goal_pose(*this->goal_pose_, *msg))
          {
            // goal mux 会周期重发同一粗目标, 只更新时间戳而不重复触发规划
            // graph 更新仍会调用 plan_to_goal, 因此不会降低环境变化后的重规划能力
            this->goal_pose_ = msg;
            return;
          }
          if (this->goal_pose_)
          {
            this->planner_.reset_exploration_state();
          }
          this->goal_pose_ = msg;
          this->plan_to_goal();
        });
    object_search_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "~/object_search_status", 10, [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          this->on_object_search_status(*msg);
        });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odom_ = msg; });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    grid_map_debug_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/unexplored_space_map", 10);
    scores_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/frontier_scores", 10);
  }

private:
  static std::string object_search_state(const std::string& status)
  {
    constexpr char prefix[] = "state=";
    if (status.rfind(prefix, 0) != 0)
    {
      return {};
    }
    const size_t separator = status.find(',');
    const size_t state_begin = sizeof(prefix) - 1;
    if (separator == std::string::npos)
    {
      return status.substr(state_begin);
    }
    return status.substr(state_begin, separator - state_begin);
  }

  void on_object_search_status(const std_msgs::msg::String& msg)
  {
    const std::string state = object_search_state(msg.data);
    if (state.empty() || state == object_search_state_)
    {
      return;
    }

    object_search_state_ = state;
    directional_exploration_mode_ = state == "SEARCHING_WITH_INITIAL_GOAL";
    // 状态切换先丢弃旧 goal, 等同一周期的新 goal 到达后再规划
    // 这样目标出现时不会用旧探索 goal 短暂发布错误路径
    goal_pose_.reset();
    planner_.reset_exploration_state();
    RCLCPP_INFO(
      this->get_logger(),
      "目标搜索规划模式切换, state=%s, directional_exploration=%s",
      state.c_str(),
      directional_exploration_mode_ ? "true" : "false");
  }

  static bool same_goal_pose(
    const geometry_msgs::msg::PoseStamped& lhs,
    const geometry_msgs::msg::PoseStamped& rhs)
  {
    if (lhs.header.frame_id != rhs.header.frame_id)
    {
      return false;
    }
    const double dx = lhs.pose.position.x - rhs.pose.position.x;
    const double dy = lhs.pose.position.y - rhs.pose.position.y;
    const double dz = lhs.pose.position.z - rhs.pose.position.z;
    const double quaternion_dot =
      lhs.pose.orientation.x * rhs.pose.orientation.x +
      lhs.pose.orientation.y * rhs.pose.orientation.y +
      lhs.pose.orientation.z * rhs.pose.orientation.z +
      lhs.pose.orientation.w * rhs.pose.orientation.w;
    return dx * dx + dy * dy + dz * dz <= 1e-8 &&
      std::abs(std::abs(quaternion_dot) - 1.0) <= 1e-6;
  }

  void plan_to_goal()
  {
    if (goal_pose_ && latest_graph_header_)
    {
      geometry_msgs::msg::PoseStamped goal = *goal_pose_;
      goal.header.stamp = latest_graph_header_->stamp;
      geometry_msgs::msg::PoseStamped goal_in_graph_frame;
      try
      {
        goal_in_graph_frame = tf_buffer_.transform(goal, latest_graph_header_->frame_id, tf2::durationFromSec(0.1));
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to graph frame: %s", ex.what());
        return;
      }
      Eigen::Vector3d goal_vec(goal_in_graph_frame.pose.position.x, goal_in_graph_frame.pose.position.y,
                               goal_in_graph_frame.pose.position.z);
      std::optional<Eigen::Vector3d> robot_position;
      if (odom_)
      {
        try
        {
          geometry_msgs::msg::PoseStamped robot_in_odom_frame;
          robot_in_odom_frame.header = odom_->header;
          robot_in_odom_frame.pose = odom_->pose.pose;
          geometry_msgs::msg::PoseStamped robot_in_graph_frame = tf_buffer_.transform(
            robot_in_odom_frame, latest_graph_header_->frame_id, tf2::durationFromSec(0.1));
          Eigen::Vector3d robot_vec(robot_in_graph_frame.pose.position.x, robot_in_graph_frame.pose.position.y,
                                    robot_in_graph_frame.pose.position.z);
          robot_position = robot_vec;
          if (directional_exploration_mode_ && !planner_.has_directional_exploration())
          {
            const auto& orientation = goal_in_graph_frame.pose.orientation;
            const Eigen::Quaterniond rotation(
              orientation.w,
              orientation.x,
              orientation.y,
              orientation.z);
            const Eigen::Vector3d heading = rotation * Eigen::Vector3d::UnitX();
            const double lookahead_distance = (goal_vec - robot_vec).head<2>().norm();
            planner_.start_directional_exploration(
              robot_vec,
              heading,
              lookahead_distance);
          }
          if (!directional_exploration_mode_ && (goal_vec - robot_vec).norm() < goal_radius_)
          {
            publish_hold_path(robot_in_graph_frame);
            goal_pose_.reset();
            return;
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform robot pose to graph frame: %s", ex.what());
        }
      }
      const auto planning_result = planner_.plan_to_goal(
        goal_vec,
        goal_radius_,
        this->get_clock()->now(),
        robot_position);
      if (planning_result.path_changed)
      {
        // Graph 高频更新只做路线验证, 仅提交分支改变或路线失效时发布新 Path
        nav_msgs::msg::Path path_msg;
        path_msg.header = *latest_graph_header_;
        path_msg.poses.resize(planning_result.path.size());
        for (size_t i = 0; i < planning_result.path.size(); i++)
        {
          path_msg.poses[i].header = path_msg.header;
          path_msg.poses[i].pose.position.x = planning_result.path[i].x();
          path_msg.poses[i].pose.position.y = planning_result.path[i].y();
          path_msg.poses[i].pose.position.z = planning_result.path[i].z();
          if (i < planning_result.path.size() - 1)
          {
            // 相邻路径点重合时保留默认朝向, 避免零向量归一化
            Eigen::Vector3d delta = planning_result.path[i + 1] - planning_result.path[i];
            if (delta.norm() < 1e-6)
            {
              path_msg.poses[i].pose.orientation.w = 1.0;
              continue;
            }
            Eigen::Vector3d direction = delta.normalized();
            Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
            Eigen::Vector3d lateral = Eigen::Vector3d::UnitZ().cross(direction);
            if (lateral.norm() < 1e-6)
            {
              path_msg.poses[i].pose.orientation.w = 1.0;
              continue;
            }
            orientation.col(1) = lateral.normalized();
            orientation.col(0) = orientation.col(1).cross(orientation.col(2)).normalized();
            Eigen::Quaterniond quaternion(orientation);
            path_msg.poses[i].pose.orientation.x = quaternion.x();
            path_msg.poses[i].pose.orientation.y = quaternion.y();
            path_msg.poses[i].pose.orientation.z = quaternion.z();
            path_msg.poses[i].pose.orientation.w = quaternion.w();
          }
          else
          {
            path_msg.poses[i].pose.orientation = goal_in_graph_frame.pose.orientation;
          }
        }
        path_pub_->publish(path_msg);
      }
      if (grid_map_debug_pub_->get_subscription_count() > 0)
      {
        grid_map_msgs::msg::GridMap grid_map_msg = planner_.get_unexplored_debug_map();
        grid_map_msg.header = *latest_graph_header_;
        grid_map_debug_pub_->publish(grid_map_msg);
      }
      if (scores_debug_pub_->get_subscription_count() > 0)
      {
        visualization_msgs::msg::MarkerArray marker_array = planner_.get_score_visualization(
          this->get_clock()->now(), latest_graph_header_->frame_id, true);
        scores_debug_pub_->publish(marker_array);
      }
      if (odom_)
      {
        try
        {
          geometry_msgs::msg::PoseStamped goal_in_odom_frame;
          goal_in_odom_frame = tf_buffer_.transform(goal, odom_->header.frame_id, tf2::durationFromSec(0.1));
          Eigen::Vector3d goal_vec(goal_in_odom_frame.pose.position.x, goal_in_odom_frame.pose.position.y,
                                   goal_in_odom_frame.pose.position.z);
          Eigen::Vector3d odom_vec(odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                                   odom_->pose.pose.position.z);
          if (!directional_exploration_mode_ && (goal_vec - odom_vec).norm() < goal_radius_)
          {
            goal_pose_.reset();  // clear goal
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to odom frame: %s", ex.what());
        }
      }
    }
  }

  void publish_hold_path(const geometry_msgs::msg::PoseStamped& robot_pose)
  {
    // 目标已在到达半径内时发布单点 path, 让自研导航立即进入停止条件
    nav_msgs::msg::Path path_msg;
    path_msg.header = *latest_graph_header_;
    geometry_msgs::msg::PoseStamped hold_pose = robot_pose;
    hold_pose.header = path_msg.header;
    path_msg.poses.push_back(hold_pose);
    path_pub_->publish(path_msg);
  }

  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_search_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scores_debug_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<std_msgs::msg::Header> latest_graph_header_;
  std::string object_search_state_;
  bool directional_exploration_mode_ = false;
  double goal_radius_;

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
