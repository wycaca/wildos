#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{

visualization_msgs::msg::MarkerArray Planner::get_score_visualization(const rclcpp::Time& stamp,
                                                                      std::string frame_id,
                                                                      bool with_id_text) const
{
  visualization_msgs::msg::MarkerArray markers;

  // delete all previous markers
  visualization_msgs::msg::Marker delete_all_infcost;
  delete_all_infcost.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_infcost.header.frame_id = frame_id;
  delete_all_infcost.header.stamp = stamp;
  delete_all_infcost.ns = "frontier_inf_cost";
  delete_all_infcost.id = 0;
  markers.markers.push_back(delete_all_infcost);

  visualization_msgs::msg::Marker delete_all_idtext;
  delete_all_idtext.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_idtext.header.frame_id = frame_id;
  delete_all_idtext.header.stamp = stamp;
  delete_all_idtext.ns = "frontier_id_text";
  delete_all_idtext.id = 0;
  markers.markers.push_back(delete_all_idtext);

  // 显示当前分支继承半径, 用于确认拐角处是否保持同一探索方向
  if (active_branch_) {
    visualization_msgs::msg::Marker frontier_marker;
    frontier_marker.header.frame_id = frame_id;
    frontier_marker.header.stamp = stamp;
    frontier_marker.ns = "local_frontier";
    frontier_marker.id = 0;
    frontier_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    frontier_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_marker.pose.position.x = active_branch_->frontier_position.x();
    frontier_marker.pose.position.y = active_branch_->frontier_position.y();
    frontier_marker.pose.position.z = active_branch_->frontier_position.z();
    frontier_marker.scale.x = frontier_continuity_radius_ * 2;
    frontier_marker.scale.y = frontier_continuity_radius_ * 2;
    frontier_marker.scale.z = 0.1;
    frontier_marker.color.a = 0.5;
    frontier_marker.color.r = 0.0;
    frontier_marker.color.g = 1.0;
    frontier_marker.color.b = 0.0;
    markers.markers.push_back(frontier_marker);
  }
  else{
    // delete
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "local_frontier";
    delete_marker.id = 0;
    markers.markers.push_back(delete_marker);
  }

  // 显示距离当前分支判定为无进展还剩多少时间
  if (active_branch_)
  {
    double time_diff = (stamp - active_branch_->progress_time).seconds();
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = stamp;
    text_marker.ns = "local_frontier_time";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = active_branch_->frontier_position.x() + frontier_continuity_radius_;
    text_marker.pose.position.y = active_branch_->frontier_position.y() + frontier_continuity_radius_;
    text_marker.pose.position.z = active_branch_->frontier_position.z() + 1.0;  // raise text above the node
    text_marker.scale.z = 1.5;           // only scale.z is used for text
    text_marker.color.a = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    std::ostringstream ss;
    ss << "Branch continuity\nTimeout: " << frontier_progress_timeout_ - time_diff;
    text_marker.text = ss.str();
    markers.markers.push_back(text_marker);
  }

  // visualize cubes for nodes with inf frontier cost
  int id_counter = 1;

  // normalize costs to [0,1] for visualization
  double min_val = 30.0;
  double max_val = 400.0;

  for (const auto& [id, score_pair] : frontier_scores_)
  {
    const auto& node = score_pair.first;
    double frontier_score = score_pair.second.first;
    double frontier_cost = score_pair.second.second;
    // if (frontier_cost == std::numeric_limits<double>::infinity() && frontier_score > 0)
    if (true)
    {
      double norm_cost = (frontier_cost - min_val) / (max_val - min_val);
      norm_cost = std::min(1.0, std::max(0.0, norm_cost));
      std_msgs::msg::ColorRGBA color = colormapJet(norm_cost);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "frontier_inf_cost";
      marker.id = id_counter;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = node.pose;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      // marker.color.a = 0.5;
      // marker.color.r = 1.0;
      // marker.color.g = 0.0;
      // marker.color.b = 0.0;
      marker.color = color;
      markers.markers.push_back(marker);
    }
    if (with_id_text)
    {
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = stamp;
      text_marker.ns = "frontier_id_text";
      text_marker.id = id_counter;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose = node.pose;
      text_marker.pose.position.z += 0.2;  // raise text above the node
      text_marker.scale.z = 0.28;          // only scale.z is used for text
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      std::ostringstream ss;
      // ss << "Score: " << frontier_score << " Cost: " << frontier_cost;
      // upto 2 decimal places
      ss << std::fixed << std::setprecision(2);
      if (std::isfinite(frontier_score) && frontier_score >= 0.0)
      {
        ss << std::clamp(frontier_score, 0.0, 1.0);
      }
      else
      {
        ss << "--";
      }
      text_marker.text = ss.str();
      markers.markers.push_back(text_marker);
    }
    id_counter++;
  }
  // }
  return markers;
}


}  // namespace graphnav_planner
