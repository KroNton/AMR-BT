#include "amr-bt/rotate_action_node.hpp"

// ── Constructor ────────────────────────────────────────────────────────────

RotateAction::RotateAction(const std::string& name,
                           const BT::NodeConfig& conf,
                           rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, conf),
    node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
}

// ── Ports ──────────────────────────────────────────────────────────────────

BT::PortsList RotateAction::providedPorts()
{
  return {
    BT::InputPort<float>("angle_degrees",    90.0f, "Target rotation in degrees"),
    BT::InputPort<float>("angular_velocity", 0.5f,  "Speed in rad/s (always positive)"),
    BT::InputPort<std::string>("direction",  "CCW", "CW or CCW")
  };
}

// ── onStart — runs once when node is first ticked ─────────────────────────

BT::NodeStatus RotateAction::onStart()
{
  float angle_deg = 90.0f;
  std::string direction = "CCW";

  getInput("angle_degrees",    angle_deg);
  getInput("angular_velocity", angular_velocity_);
  getInput("direction",        direction);

  // validate
  if (angular_velocity_ <= 0.0f) {
    RCLCPP_ERROR(node_->get_logger(),
      "[RotateAction] angular_velocity must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (angle_deg <= 0.0f) {
    RCLCPP_ERROR(node_->get_logger(),
      "[RotateAction] angle_degrees must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  // convert degrees → radians
  target_angle_rad_ = angle_deg * M_PI / 180.0f;

  // direction: CCW = positive angular.z, CW = negative
  direction_sign_ = (direction == "CW") ? -1.0f : 1.0f;

  // reset accumulator
  angle_rotated_rad_ = 0.0f;
  last_tick_time_    = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[RotateAction] Starting: %.1f deg | %.2f rad/s | %s",
    angle_deg, angular_velocity_, direction.c_str());

  // start moving immediately
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = direction_sign_ * angular_velocity_;
  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

// ── onRunning — called every tick until done ───────────────────────────────

BT::NodeStatus RotateAction::onRunning()
{
  // compute elapsed time since last tick
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_tick_time_).count();
  last_tick_time_ = now;

  // accumulate angle rotated
  angle_rotated_rad_ += angular_velocity_ * dt;

  RCLCPP_DEBUG(node_->get_logger(),
    "[RotateAction] rotated: %.3f / %.3f rad",
    angle_rotated_rad_, target_angle_rad_);

  // check if we reached the target
  if (angle_rotated_rad_ >= target_angle_rad_) {
    stopRobot();
    RCLCPP_INFO(node_->get_logger(), "[RotateAction] Rotation complete.");
    return BT::NodeStatus::SUCCESS;
  }

  // keep spinning
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = direction_sign_ * angular_velocity_;
  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

// ── onHalted — tree aborted this node ─────────────────────────────────────

void RotateAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(),
    "[RotateAction] Halted — stopping robot.");
  stopRobot();
}

// ── Private ────────────────────────────────────────────────────────────────

void RotateAction::stopRobot()
{
  geometry_msgs::msg::Twist stop;
  stop.angular.z = 0.0;
  stop.linear.x  = 0.0;
  cmd_vel_pub_->publish(stop);
}