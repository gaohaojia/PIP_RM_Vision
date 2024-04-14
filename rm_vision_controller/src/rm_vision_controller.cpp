// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_vision_controller/rm_vision_controller.hpp"

namespace rm_vision_controller
{
RMVisionController::RMVisionController(const rclcpp::NodeOptions & options)
: Node("rm_vision_controller", options)
{
  RCLCPP_INFO(get_logger(), "Start RMVisionController!");

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  vision_target_pub_ = this->create_publisher<serial_interfaces::msg::VisionTarget>("/vision_target", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  vision_recv_sub_ = this->create_subscription<serial_interfaces::msg::VisionRecv>(
    "/vision_recv", 10, std::bind(&RMVisionController::receiveDataVision, this, std::placeholders::_1));
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMVisionController::sendDataVision, this, std::placeholders::_1));
}

void RMVisionController::receiveDataVision(const serial_interfaces::msg::VisionRecv::SharedPtr msg)
{
  if (!initial_set_param_ || msg->detect_color != previous_receive_color_) {
    setParam(rclcpp::Parameter("detect_color", msg->detect_color));
    previous_receive_color_ = msg->detect_color;
  }

  if (msg->reset_tracker) {
    resetTracker();
  }

  geometry_msgs::msg::TransformStamped t;
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
  t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  t.header.frame_id = "odom";
  t.child_frame_id = "gimbal_link";
  tf2::Quaternion q;
  q.setRPY(msg->roll, msg->pitch, msg->yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  if (abs(msg->aim_x) > 0.01) {
    aiming_point_.header.stamp = this->now();
    aiming_point_.pose.position.x = msg->aim_x;
    aiming_point_.pose.position.y = msg->aim_y;
    aiming_point_.pose.position.z = msg->aim_z;
    marker_pub_->publish(aiming_point_);
  }
}

void RMVisionController::sendDataVision(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  serial_interfaces::msg::VisionTarget vision_target;
  vision_target.tracking = msg->tracking;
  vision_target.id = msg->id;
  vision_target.armors_num = msg->armors_num;
  vision_target.position = msg->position;
  vision_target.velocity = msg->velocity;
  vision_target.yaw = msg->yaw;
  vision_target.v_yaw = msg->v_yaw;
  vision_target.radius_1 = msg->radius_1;
  vision_target.radius_2 = msg->radius_2;
  vision_target.dz = msg->dz;
  vision_target_pub_->publish(vision_target);

  std_msgs::msg::Float64 latency;
  latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
  RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
  latency_pub_->publish(latency);
}


void RMVisionController::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMVisionController::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_vision_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_vision_controller::RMVisionController)