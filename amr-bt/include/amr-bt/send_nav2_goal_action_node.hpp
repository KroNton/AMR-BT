#pragma once

#include <string>
#include <memory>

#include <behaviortree_ros2/bt_action_node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class SendWaypointAction : public BT::RosActionNode<NavigateToPose>
{
public:
  SendWaypointAction(const std::string& name,
                     const BT::NodeConfig& conf,
                     const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setGoal(RosActionNode::Goal& goal) override;

  BT::NodeStatus onFeedback(
    const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  geometry_msgs::msg::PoseStamped make_goal_pose(
    float pose_x, float pose_y, float pose_yaw);

  float goal_pose_x_  = 0.0f;
  float goal_pose_y_  = 0.0f;
  float goal_pose_yaw_= 0.0f;
};