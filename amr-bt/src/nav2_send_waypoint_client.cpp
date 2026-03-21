#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using NavigateToPose  = nav2_msgs::action::NavigateToPose;
using GoalHandle      = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;


static constexpr double GOAL_X        =  -0.356;   // metres
static constexpr double GOAL_Y        =  5.2;   // metres
static constexpr double GOAL_YAW_DEG  = 90.0;   // degrees (yaw only)
static constexpr double TIMEOUT_SEC   = 30.0;   // seconds before cancel

class Nav2SendWaypointClient : public rclcpp::Node
{
public:
  Nav2SendWaypointClient()
  : Node("nav2_send_waypoint_client"), goal_done_(false)
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // Give Nav2 action server time to come up, then send the goal
    timer_ = create_wall_timer(
      500ms, std::bind(&Nav2SendWaypointClient::send_goal, this));
  }

  bool is_goal_done() const { return goal_done_; }

private:
  // ── members ──────────────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr                     timer_;
  rclcpp::TimerBase::SharedPtr                     timeout_timer_;
  GoalHandle::SharedPtr                            goal_handle_;
  bool                                             goal_done_;

  geometry_msgs::msg::PoseStamped make_goal_pose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = now();
    pose.header.frame_id = "map";

    pose.pose.position.x = GOAL_X;
    pose.pose.position.y = GOAL_Y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, GOAL_YAW_DEG * M_PI / 180.0);
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }

  // ── send goal ─────────────────────────────────────────────────────────────
  void send_goal()
  {
    timer_->cancel();   // fire only once

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(),
        "navigate_to_pose action server not available after 5 s. Shutting down.");
      goal_done_ = true;
      return;
    }

    auto goal_msg          = NavigateToPose::Goal();
    goal_msg.pose          = make_goal_pose();
    goal_msg.behavior_tree = "";   // use Nav2 default BT

    RCLCPP_INFO(get_logger(),
      "Sending goal → x=%.2f  y=%.2f  yaw=%.1f°",
      GOAL_X, GOAL_Y, GOAL_YAW_DEG);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_opts.goal_response_callback =
      std::bind(&Nav2SendWaypointClient::goal_response_cb, this, std::placeholders::_1);
    send_opts.feedback_callback =
      std::bind(&Nav2SendWaypointClient::feedback_cb, this,
        std::placeholders::_1, std::placeholders::_2);
    send_opts.result_callback =
      std::bind(&Nav2SendWaypointClient::result_cb, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_opts);
  }

  // ── callbacks ─────────────────────────────────────────────────────────────
  void goal_response_cb(const GoalHandle::SharedPtr & handle)
  {
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "Goal was REJECTED by the server.");
      goal_done_ = true;
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal ACCEPTED. Timeout set to %.0f s.", TIMEOUT_SEC);
    goal_handle_ = handle;

  }

  void feedback_cb(
    GoalHandle::SharedPtr /*handle*/,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(),
      "[feedback] dist to goal: %.2f m | ETA: %.1f s | recoveries: %u",
      feedback->distance_remaining,
      feedback->estimated_time_remaining.sec +
        feedback->estimated_time_remaining.nanosec * 1e-9,
      feedback->number_of_recoveries);
  }

  void result_cb(const GoalHandle::WrappedResult & result)
  {
    if (timeout_timer_) timeout_timer_->cancel();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), " Goal SUCCEEDED!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), " Goal ABORTED by server.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), " Goal CANCELED.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code: %d",
          static_cast<int>(result.code));
    }

    goal_done_ = true;
  }

};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Nav2SendWaypointClient>();

  // Spin until the goal is done (succeeded, failed, or timed-out)
  while (rclcpp::ok() && !node->is_goal_done()) {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}