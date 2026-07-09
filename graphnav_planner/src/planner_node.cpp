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
#include <optional>
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
    this->declare_parameter("frontier_score_factor", 10.0);
    this->declare_parameter("append_virtual_goal_to_path", false);
    this->declare_parameter("append_frontier_point_to_path", false);
    this->declare_parameter("min_local_frontier_score", 0.4);
    this->declare_parameter("local_frontier_radius", 7.0);
    this->declare_parameter("path_smoothness_period", 10.0);
    this->declare_parameter("path_switch_hysteresis_sec", 6.0);
    this->declare_parameter("path_switch_min_endpoint_delta", 3.0);
    this->declare_parameter("path_switch_min_goal_delta", 5.0);

    planner_.frontier_dist_cost_factor_ = this->get_parameter("frontier_dist_cost_factor").as_double();
    planner_.goal_dist_cost_factor_ = this->get_parameter("goal_dist_cost_factor").as_double();
    planner_.frontier_score_factor_ = this->get_parameter("frontier_score_factor").as_double();
    planner_.append_virtual_goal_to_path_ = this->get_parameter("append_virtual_goal_to_path").as_bool();
    planner_.append_frontier_point_to_path_ = this->get_parameter("append_frontier_point_to_path").as_bool();
    planner_.min_local_frontier_score_ = this->get_parameter("min_local_frontier_score").as_double();
    planner_.local_frontier_radius_ = this->get_parameter("local_frontier_radius").as_double();
    planner_.path_smoothness_period_ = this->get_parameter("path_smoothness_period").as_double();
    path_switch_hysteresis_sec_ = this->get_parameter("path_switch_hysteresis_sec").as_double();
    path_switch_min_endpoint_delta_ = this->get_parameter("path_switch_min_endpoint_delta").as_double();
    path_switch_min_goal_delta_ = this->get_parameter("path_switch_min_goal_delta").as_double();

    this->declare_parameter("trav_class", "default");
    planner_.set_trav_class(this->get_parameter("trav_class").as_string());

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
          this->goal_pose_ = msg;
          this->plan_to_goal();
        });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odom_ = msg; });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    grid_map_debug_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/unexplored_space_map", 10);
    scores_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/frontier_scores", 10);
  }

private:
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
          if ((goal_vec - robot_vec).norm() < goal_radius_)
          {
            publish_hold_path(robot_in_graph_frame, goal_vec, this->get_clock()->now());
            goal_pose_.reset();
            return;
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform robot pose to graph frame: %s", ex.what());
        }
      }
      auto path = planner_.plan_to_goal(goal_vec, goal_radius_, this->get_clock()->now());
      nav_msgs::msg::Path path_msg;
      path_msg.header = *latest_graph_header_;
      path_msg.poses.resize(path.size());
      for (size_t i = 0; i < path.size(); i++)
      {
        path_msg.poses[i].header = path_msg.header;
        path_msg.poses[i].pose.position.x = path[i].x();
        path_msg.poses[i].pose.position.y = path[i].y();
        path_msg.poses[i].pose.position.z = path[i].z();
        if (i < path.size() - 1)
        {
          // 相邻路径点重合时保留默认朝向, 避免零向量归一化
          Eigen::Vector3d delta = path[i + 1] - path[i];
          if (delta.norm() < 1e-6)
          {
            path_msg.poses[i].pose.orientation.w = 1.0;
            continue;
          }
          Eigen::Vector3d d = delta.normalized();
          Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
          Eigen::Vector3d lateral = Eigen::Vector3d::UnitZ().cross(d);
          if (lateral.norm() < 1e-6)
          {
            path_msg.poses[i].pose.orientation.w = 1.0;
            continue;
          }
          m.col(1) = lateral.normalized();
          m.col(0) = m.col(1).cross(m.col(2)).normalized();
          Eigen::Quaterniond q(m);
          path_msg.poses[i].pose.orientation.x = q.x();
          path_msg.poses[i].pose.orientation.y = q.y();
          path_msg.poses[i].pose.orientation.z = q.z();
          path_msg.poses[i].pose.orientation.w = q.w();
        }
        else  // last waypoint
        {
          path_msg.poses[i].pose.orientation = goal_in_graph_frame.pose.orientation;
        }
      }
      auto now = this->get_clock()->now();
      if (should_keep_previous_path(path_msg, goal_vec, now))
      {
        nav_msgs::msg::Path stable_path = retime_path(last_path_msg_, *latest_graph_header_);
        path_pub_->publish(stable_path);
        return;
      }
      path_pub_->publish(path_msg);
      remember_path(path_msg, goal_vec, now);
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
          if ((goal_vec - odom_vec).norm() < goal_radius_)
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

  bool should_keep_previous_path(const nav_msgs::msg::Path& new_path,
                                 const Eigen::Vector3d& goal,
                                 const rclcpp::Time& now)
  {
    if (!has_last_path_ || path_switch_hysteresis_sec_ <= 0.0)
    {
      return false;
    }
    if (!last_path_time_ || !last_path_goal_)
    {
      return false;
    }
    if ((now - *last_path_time_).seconds() > path_switch_hysteresis_sec_)
    {
      return false;
    }
    if ((goal - *last_path_goal_).norm() > path_switch_min_goal_delta_)
    {
      return false;
    }
    if (last_path_msg_.poses.empty())
    {
      return false;
    }
    if (new_path.poses.empty())
    {
      return !robot_is_near_path_end(last_path_msg_);
    }
    Eigen::Vector3d old_end = pose_position(last_path_msg_.poses.back());
    Eigen::Vector3d new_end = pose_position(new_path.poses.back());
    if ((old_end - new_end).norm() < path_switch_min_endpoint_delta_)
    {
      return false;
    }
    return !robot_is_near_path_end(last_path_msg_);
  }

  bool robot_is_near_path_end(const nav_msgs::msg::Path& path) const
  {
    if (!odom_ || path.poses.empty())
    {
      return false;
    }
    Eigen::Vector3d end = pose_position(path.poses.back());
    Eigen::Vector3d robot(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
    return (end - robot).norm() <= goal_radius_;
  }

  void remember_path(const nav_msgs::msg::Path& path, const Eigen::Vector3d& goal, const rclcpp::Time& now)
  {
    if (path.poses.empty())
    {
      return;
    }
    last_path_msg_ = path;
    last_path_time_ = now;
    last_path_goal_ = goal;
    has_last_path_ = true;
  }

  void publish_hold_path(const geometry_msgs::msg::PoseStamped& robot_pose,
                         const Eigen::Vector3d& goal,
                         const rclcpp::Time& now)
  {
    // 目标已在到达半径内时发布单点 path, 让外部 path follower 立即进入停止条件
    nav_msgs::msg::Path path_msg;
    path_msg.header = *latest_graph_header_;
    geometry_msgs::msg::PoseStamped hold_pose = robot_pose;
    hold_pose.header = path_msg.header;
    path_msg.poses.push_back(hold_pose);
    path_pub_->publish(path_msg);
    remember_path(path_msg, goal, now);
  }

  nav_msgs::msg::Path retime_path(const nav_msgs::msg::Path& path, const std_msgs::msg::Header& header) const
  {
    nav_msgs::msg::Path retimed = path;
    retimed.header = header;
    for (auto& pose : retimed.poses)
    {
      pose.header = header;
    }
    return retimed;
  }

  static Eigen::Vector3d pose_position(const geometry_msgs::msg::PoseStamped& pose)
  {
    return Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }

  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scores_debug_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<std_msgs::msg::Header> latest_graph_header_;
  double goal_radius_;
  double path_switch_hysteresis_sec_;
  double path_switch_min_endpoint_delta_;
  double path_switch_min_goal_delta_;
  nav_msgs::msg::Path last_path_msg_;
  std::optional<rclcpp::Time> last_path_time_;
  std::optional<Eigen::Vector3d> last_path_goal_;
  bool has_last_path_ = false;

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
