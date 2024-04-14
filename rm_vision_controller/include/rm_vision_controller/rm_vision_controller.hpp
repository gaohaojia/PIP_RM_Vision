// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_VISION_CONTROLLER__RM_VISION_CONTROLLER_HPP_
#define RM_VISION_CONTROLLER__RM_VISION_CONTROLLER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <serial_interfaces/msg/referee.hpp>
#include <serial_interfaces/msg/vision_target.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <serial_interfaces/msg/vision_recv.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_vision_controller
{
class RMVisionController : public rclcpp::Node
{
public:
  explicit RMVisionController(const rclcpp::NodeOptions & options);

private:

  void receiveDataVision(serial_interfaces::msg::VisionRecv::SharedPtr msg);
  void sendDataVision(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void setParam(const rclcpp::Parameter & param);

  void resetTracker();

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<serial_interfaces::msg::VisionRecv>::SharedPtr vision_recv_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<serial_interfaces::msg::VisionTarget>::SharedPtr vision_target_pub_;
};
}  // namespace rm_vision_controller

#endif  // RM_VISION_CONTROLLER__RM_VISION_CONTROLLER_HPP_