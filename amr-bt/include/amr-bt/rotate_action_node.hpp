#pragma once

#include <chrono>
#include <string>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RotateAction : public BT::StatefulActionNode
{
public:
  RotateAction(const std::string& name,
               const BT::NodeConfig& conf,
               rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // called once when node becomes RUNNING
  BT::NodeStatus onStart() override;

  // called every tick while RUNNING
  BT::NodeStatus onRunning() override;

  // called if the tree aborts this node
  void onHalted() override;

private:
  void stopRobot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  float  target_angle_rad_  = 0.0f;  // converted from degrees
  float  angular_velocity_  = 0.0f;  // rad/s — always positive
  float  direction_sign_    = 1.0f;  // +1 CCW, -1 CW

  float  angle_rotated_rad_ = 0.0f;  // accumulated so far
  std::chrono::steady_clock::time_point last_tick_time_;
};