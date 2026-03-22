#include <chrono>
#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using NavigateToPose  = nav2_msgs::action::NavigateToPose;
using GoalHandle      = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using namespace BT;

class SendWaypointAction: public RosActionNode<NavigateToPose>
{

private:
  float goal_pose_x_,goal_pose_y_,goal_pose_yaw_;

public:
  SendWaypointAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<NavigateToPose>(name, conf, params)
  {

  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<float>("pose_x"),
      InputPort<float>("pose_y"),
      InputPort<float>("pose_yaw"),
    
    });
  }

  geometry_msgs::msg::PoseStamped make_goal_pose(float pose_x, float pose_y, float pose_yaw )
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = now();
    pose.header.frame_id = "map";

    pose.pose.position.x = pose_x;
    pose.pose.position.y = pose_y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw * M_PI / 180.0);
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }
  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    // get "order" from the Input port
    getInput("pose_x", goal_pose_x_);
    getInput("pose_y", goal_pose_y_);
    getInput("pose_yaw", goal_pose_yaw_);
    
    goal.pose = make_goal_pose(goal_pose_x_,goal_pose_y_,goal_pose_yaw_);
    
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_INFO(logger(),
      "[feedback] dist to goal: %.2f m | ETA: %.1f s | recoveries: %u",
      feedback->distance_remaining,
      feedback->estimated_time_remaining.sec +
        feedback->estimated_time_remaining.nanosec * 1e-9,
      feedback->number_of_recoveries);
    return NodeStatus::RUNNING;
  }
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(logger(), " Goal SUCCEEDED!");
      return NodeStatus::SUCCESS;
    }
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.

};