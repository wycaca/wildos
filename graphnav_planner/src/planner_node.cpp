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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>
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
    this->declare_parameter("frontier_progress_start_grace", 20.0);
    this->declare_parameter("directional_min_forward_progress", 0.5);
    this->declare_parameter("directional_max_initial_backtrack", 2.0);
    this->declare_parameter("directional_block_confirm_timeout", 5.0);
    this->declare_parameter("frontier_failure_cooldown", 60.0);
    this->declare_parameter("frontier_failure_merge_radius", 2.5);
    this->declare_parameter("path_invalid_confirm_duration", 1.5);
    this->declare_parameter("path_invalid_confirm_frames", 3);
    this->declare_parameter("recovery_observe_duration", 3.0);
    this->declare_parameter("dead_end_backtrack_step", 2.0);
    this->declare_parameter("dead_end_backtrack_step_duration", 5.0);
    this->declare_parameter("dead_end_max_backtrack", 10.0);
    this->declare_parameter("deferred_branch_cost_penalty", 3.0);
    this->declare_parameter("revisit_cost_factor", 1.0);
    this->declare_parameter("max_graph_age_sec", 2.0);
    this->declare_parameter("max_odom_age_sec", 1.0);
    this->declare_parameter("odom_reset_distance", 3.0);
    this->declare_parameter("odom_reset_speed", 12.0);
    this->declare_parameter("diagnostics_log_period_sec", 30.0);
    this->declare_parameter("slow_planning_warning_ms", 200.0);

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
    planner_.frontier_progress_start_grace_ = nonnegative_parameter("frontier_progress_start_grace");
    planner_.directional_min_forward_progress_ =
      nonnegative_parameter("directional_min_forward_progress");
    planner_.directional_max_initial_backtrack_ =
      nonnegative_parameter("directional_max_initial_backtrack");
    planner_.directional_block_confirm_timeout_ =
      nonnegative_parameter("directional_block_confirm_timeout");
    planner_.frontier_failure_cooldown_ =
      nonnegative_parameter("frontier_failure_cooldown");
    planner_.frontier_failure_merge_radius_ =
      nonnegative_parameter("frontier_failure_merge_radius");
    planner_.path_invalid_confirm_duration_ =
      nonnegative_parameter("path_invalid_confirm_duration");
    const auto invalid_confirm_frames =
      this->get_parameter("path_invalid_confirm_frames").as_int();
    if (invalid_confirm_frames <= 0)
    {
      throw std::invalid_argument("path_invalid_confirm_frames must be positive");
    }
    planner_.path_invalid_confirm_frames_ =
      static_cast<size_t>(invalid_confirm_frames);
    planner_.recovery_observe_duration_ =
      nonnegative_parameter("recovery_observe_duration");
    planner_.dead_end_backtrack_step_ =
      nonnegative_parameter("dead_end_backtrack_step");
    planner_.dead_end_backtrack_step_duration_ =
      nonnegative_parameter("dead_end_backtrack_step_duration");
    planner_.dead_end_max_backtrack_ =
      nonnegative_parameter("dead_end_max_backtrack");
    planner_.deferred_branch_cost_penalty_ =
      nonnegative_parameter("deferred_branch_cost_penalty");
    planner_.revisit_cost_factor_ = nonnegative_parameter("revisit_cost_factor");
    max_graph_age_sec_ = nonnegative_parameter("max_graph_age_sec");
    max_odom_age_sec_ = nonnegative_parameter("max_odom_age_sec");
    odom_reset_distance_ = nonnegative_parameter("odom_reset_distance");
    odom_reset_speed_ = nonnegative_parameter("odom_reset_speed");

    planner_.set_trav_class("default");

    this->declare_parameter("goal_radius", 3.0);
    goal_radius_ = this->get_parameter("goal_radius").as_double();
    diagnostics_log_period_sec_ = std::max(
      this->get_parameter("diagnostics_log_period_sec").as_double(), 5.0);
    slow_planning_warning_ms_ = std::max(
      this->get_parameter("slow_planning_warning_ms").as_double(), 1.0);

    graph_sub_ = this->create_subscription<graphnav_msgs::msg::NavigationGraph>(
        "~/nav_graph", 10, [this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg) {
          this->planner_.update_graph(msg);
          this->latest_graph_header_ = msg->header;
          this->plan_to_goal();
        });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          if (this->last_hold_goal_ && same_goal_pose(*this->last_hold_goal_, *msg))
          {
            return;
          }
          this->last_hold_goal_.reset();
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
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          this->on_odom(msg);
        });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    grid_map_debug_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/unexplored_space_map", 10);
    scores_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/frontier_scores", 10);
    diagnostics_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(diagnostics_log_period_sec_),
      [this]() { this->report_diagnostics(); });
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

  static const char* object_search_state_name(const std::string& state)
  {
    if (state == "WAIT_FOR_ODOM")
    {
      return "等待里程计";
    }
    if (state == "SEARCHING_WITH_INITIAL_GOAL")
    {
      return "按初始方向探索";
    }
    if (state == "TARGET_APPROACH_COARSE")
    {
      return "接近视觉粗目标";
    }
    if (state == "TARGET_APPROACH_METRIC")
    {
      return "接近稳定融合目标";
    }
    if (state == "TARGET_REACHED_VIEWPOINT")
    {
      return "目标到达观察点";
    }
    return "未知状态";
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
    last_hold_goal_.reset();
    planner_.reset_exploration_state();
    RCLCPP_INFO(
      this->get_logger(),
      "目标搜索规划模式切换, 状态=%s(%s), 路线类型=%s",
      object_search_state_name(state),
      state.c_str(),
      directional_exploration_mode_ ? "初始方向探索" : "目标接近");
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

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
  {
    if (odom_)
    {
      const rclcpp::Time current_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
      const rclcpp::Time previous_stamp(
        odom_->header.stamp,
        this->get_clock()->get_clock_type());
      const double dt = (current_stamp - previous_stamp).seconds();
      const double dx = msg->pose.pose.position.x - odom_->pose.pose.position.x;
      const double dy = msg->pose.pose.position.y - odom_->pose.pose.position.y;
      const double dz = msg->pose.pose.position.z - odom_->pose.pose.position.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double speed = dt > 1e-3 ? distance / dt :
        std::numeric_limits<double>::infinity();
      const bool clock_reset = dt < -0.1;
      const bool pose_reset = distance >= odom_reset_distance_ &&
        (dt <= 0.0 || speed >= odom_reset_speed_);
      if (clock_reset || pose_reset)
      {
        // Unity 手动重置会改变时间或位姿, 旧探索状态不能跨重置继续使用
        planner_.reset_exploration_state();
        goal_pose_.reset();
        last_hold_goal_.reset();
        latest_graph_header_.reset();
        manual_reset_events_++;
        RCLCPP_WARN(
          this->get_logger(),
          "检测到 Unity 重置或 odom 跳变, 已清理探索状态, distance=%.2fm, dt=%.3fs, speed=%.2fm/s",
          distance,
          dt,
          speed);
      }
    }
    odom_ = msg;
  }

  bool planning_inputs_healthy(double& graph_age, double& odom_age)
  {
    graph_age = std::numeric_limits<double>::infinity();
    odom_age = std::numeric_limits<double>::infinity();
    if (!latest_graph_header_ || !odom_)
    {
      return false;
    }
    const rclcpp::Time now = this->get_clock()->now();
    graph_age = (now - rclcpp::Time(
      latest_graph_header_->stamp,
      now.get_clock_type())).seconds();
    odom_age = (now - rclcpp::Time(
      odom_->header.stamp,
      now.get_clock_type())).seconds();
    constexpr double future_tolerance = 0.1;
    return graph_age >= -future_tolerance && graph_age <= max_graph_age_sec_ &&
      odom_age >= -future_tolerance && odom_age <= max_odom_age_sec_;
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
        RCLCPP_WARN(this->get_logger(), "目标位姿无法转换到导航图坐标系, 原因=%s", ex.what());
        return;
      }
      Eigen::Vector3d goal_vec(goal_in_graph_frame.pose.position.x, goal_in_graph_frame.pose.position.y,
                               goal_in_graph_frame.pose.position.z);
      std::optional<Eigen::Vector3d> robot_position;
      std::optional<geometry_msgs::msg::PoseStamped> robot_pose_for_hold;
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
          robot_pose_for_hold = robot_in_graph_frame;
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
            last_hold_goal_ = goal;
            publish_hold_path(robot_in_graph_frame, "goal_within_radius");
            goal_pose_.reset();
            return;
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "机器人位姿无法转换到导航图坐标系, 原因=%s", ex.what());
        }
      }
      const auto planning_started = std::chrono::steady_clock::now();
      double graph_age = 0.0;
      double odom_age = 0.0;
      const bool timing_inputs_healthy = planning_inputs_healthy(graph_age, odom_age);
      if (!timing_inputs_healthy)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          10000,
          "规划输入不新鲜, 已冻结分支失败计时, graph_age=%.3fs, odom_age=%.3fs",
          graph_age,
          odom_age);
      }
      const auto planning_result = planner_.plan_to_goal(
        goal_vec,
        goal_radius_,
        this->get_clock()->now(),
        robot_position,
        timing_inputs_healthy);
      const double planning_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_started).count();
      record_planning_timing(planning_ms);
      if (planning_result.path_changed)
      {
        path_changes_++;
        if (planning_result.path.empty() && robot_pose_for_hold)
        {
          empty_paths_++;
          publish_hold_path(*robot_pose_for_hold, "no_valid_route");
          return;
        }
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
        RCLCPP_INFO(
          this->get_logger(),
          "已发布规划路径, type=route, poses=%zu, frame=%s",
          path_msg.poses.size(),
          path_msg.header.frame_id.c_str());
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
            goal_pose_.reset();  // 清除已到达目标
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "目标位姿无法转换到里程计坐标系, 原因=%s", ex.what());
        }
      }
    }
  }

  void publish_hold_path(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const char* reason)
  {
    // 目标已在到达半径内时发布单点 path, 让自研导航立即进入停止条件
    nav_msgs::msg::Path path_msg;
    path_msg.header = *latest_graph_header_;
    geometry_msgs::msg::PoseStamped hold_pose = robot_pose;
    hold_pose.header = path_msg.header;
    path_msg.poses.push_back(hold_pose);
    path_pub_->publish(path_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "已发布停止路径, type=hold, reason=%s, poses=%zu, frame=%s",
      reason,
      path_msg.poses.size(),
      path_msg.header.frame_id.c_str());
  }

  void record_planning_timing(double elapsed_ms)
  {
    planning_timings_ms_.push_back(elapsed_ms);
    if (planning_timings_ms_.size() > 512)
    {
      planning_timings_ms_.pop_front();
    }
    planning_calls_++;
    const auto now = std::chrono::steady_clock::now();
    if (
      elapsed_ms >= slow_planning_warning_ms_ &&
      now - last_slow_warning_ >= std::chrono::seconds(30))
    {
      last_slow_warning_ = now;
      RCLCPP_WARN(
        this->get_logger(),
        "路径规划耗时偏高, planning=%.1fms, threshold=%.1fms",
        elapsed_ms,
        slow_planning_warning_ms_);
    }
  }

  void report_diagnostics()
  {
    if (planning_timings_ms_.empty())
    {
      return;
    }
    std::vector<double> samples(planning_timings_ms_.begin(), planning_timings_ms_.end());
    std::sort(samples.begin(), samples.end());
    const size_t p95_index = std::min(
      static_cast<size_t>(std::ceil(samples.size() * 0.95)) - 1,
      samples.size() - 1);
    const double average = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    const double rate = planning_calls_ / diagnostics_log_period_sec_;
    RCLCPP_INFO(
      this->get_logger(),
      "路径规划性能, 频率=%.2fHz, 规划耗时=平均%.1f/95%%上限%.1f/最大%.1fms, "
      "路线变化=%zu次, 空路线=%zu次, Unity重置=%zu次",
      rate,
      average,
      samples[p95_index],
      samples.back(),
      path_changes_,
      empty_paths_,
      manual_reset_events_);
    planning_timings_ms_.clear();
    planning_calls_ = 0;
    path_changes_ = 0;
    empty_paths_ = 0;
    manual_reset_events_ = 0;
  }

  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_search_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scores_debug_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> last_hold_goal_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<std_msgs::msg::Header> latest_graph_header_;
  std::string object_search_state_;
  bool directional_exploration_mode_ = false;
  double goal_radius_;
  double diagnostics_log_period_sec_;
  double slow_planning_warning_ms_;
  double max_graph_age_sec_;
  double max_odom_age_sec_;
  double odom_reset_distance_;
  double odom_reset_speed_;
  std::deque<double> planning_timings_ms_;
  size_t planning_calls_ = 0;
  size_t path_changes_ = 0;
  size_t empty_paths_ = 0;
  size_t manual_reset_events_ = 0;
  std::chrono::steady_clock::time_point last_slow_warning_{};

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
