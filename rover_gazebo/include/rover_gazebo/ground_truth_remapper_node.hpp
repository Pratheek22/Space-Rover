#ifndef GROUND_TRUTH_REMAPPER_NODE_HPP
#define GROUND_TRUTH_REMAPPER_NODE_HPP

#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GroundTruthRemapperNode : public rclcpp::Node {
public:
  GroundTruthRemapperNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);
  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  bool initial_pose_set_;
  double initial_x_;
  double initial_y_;
  double initial_z_;
  double initial_yaw_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

#endif // GROUND_TRUTH_REMAPPER_NODE_HPP
