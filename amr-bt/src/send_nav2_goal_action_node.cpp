#include "amr-bt/send_nav2_goal_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
// ── Constructor ────────────────────────────────────────────────────────────

SendWaypointAction::SendWaypointAction(const std::string& name,
                                       const BT::NodeConfig& conf,
                                       const BT::RosNodeParams& params)
  : RosActionNode<NavigateToPose>(name, conf, params)
{

  
}

// ── Ports ──────────────────────────────────────────────────────────────────

BT::PortsList SendWaypointAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<float>("pose_x"),
    BT::InputPort<float>("pose_y"),
    BT::InputPort<float>("pose_yaw")
  });
}

// ── Private helpers ────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped SendWaypointAction::make_goal_pose(
  float pose_x, float pose_y, float pose_yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp    = rclcpp::Clock(RCL_ROS_TIME).now();
  pose.header.frame_id = "map";

  pose.pose.position.x = pose_x;
  pose.pose.position.y = pose_y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_yaw * M_PI / 180.0);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

// ── BT overrides ───────────────────────────────────────────────────────────

bool SendWaypointAction::setGoal(RosActionNode::Goal& goal)
{
  getInput("pose_x",   goal_pose_x_);
  getInput("pose_y",   goal_pose_y_);
  getInput("pose_yaw", goal_pose_yaw_);

  goal.pose = make_goal_pose(goal_pose_x_, goal_pose_y_, goal_pose_yaw_);
  return true;
}

BT::NodeStatus SendWaypointAction::onFeedback(
  const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(),
    "[feedback] dist: %.2f m | ETA: %.1f s | recoveries: %u",
    feedback->distance_remaining,
    feedback->estimated_time_remaining.sec +
      feedback->estimated_time_remaining.nanosec * 1e-9,
    feedback->number_of_recoveries);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendWaypointAction::onResultReceived(const WrappedResult& wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Goal SUCCEEDED!");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(logger(), "Goal ended with unexpected code: %d",
    static_cast<int>(wr.code));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SendWaypointAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Action failed — error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<SendWaypointAction>(
    "SendWaypointAction",
    [](const std::string& name, const BT::NodeConfig& conf)
    {
      // read node handle from blackboard — set by main.cpp
      auto node = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");

      BT::RosNodeParams params;
      params.nh = node;
      params.default_port_value    = "navigate_to_pose";
      params.server_timeout        = std::chrono::milliseconds(5000);
      params.wait_for_server_timeout = std::chrono::milliseconds(10000);

      return std::make_unique<SendWaypointAction>(name, conf, params);
    });
}