#pragma once
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sim_common.h"
#include "arm.h"
#include "freeflying.h"
#include "car.h"
#include "world.h"

class RosBridge : public rclcpp::Node {
public:
  RosBridge(World& world, std::vector<RobotHandle>& robots);

  // Call this once per GUI frame (non-blocking)
  void spinOnce();

private:
  World& world_;
  std::vector<RobotHandle>& robots_;

  struct PerRobot {
    RobotHandle* h{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_cmd;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gripper;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;
  };
  std::vector<PerRobot> pr_;

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg, PerRobot& pr);
  void onJointCmd(const sensor_msgs::msg::JointState::SharedPtr msg, PerRobot& pr);
  void onGripper(const std_msgs::msg::Bool::SharedPtr msg, PerRobot& pr);

  void publishJointStates();
};