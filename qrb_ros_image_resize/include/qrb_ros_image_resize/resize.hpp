// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_IMAGE_RESIZE__RESIZE_HPP_
#define QRB_ROS_IMAGE_RESIZE__RESIZE_HPP_

#include "cv_bridge/cv_bridge.h"
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "qrb_ros_transport/type/image.hpp"

#ifdef EVA_ENABLE
#include "qrb_image_resize_lib/eva_utils_lib.hpp"
#endif

#define MAX_HEIGHT 2160
#define MAX_WIDTH 3840
#define MIN_DST 64
#define LIMIT_SCALE 0.125

#define CALCULATE_TIME 5

namespace qrb_ros::resize
{
class ResizeNode : public rclcpp::Node
{
public:
  explicit ResizeNode(const rclcpp::NodeOptions &);

private:
  std::shared_ptr<rclcpp::Subscription<qrb_ros::transport::type::Image>> image_sub_;
  std::shared_ptr<rclcpp::Publisher<qrb_ros::transport::type::Image>> image_pub_;

  int interpolation_;
  bool use_scale_;
  double scale_height_;
  double scale_width_;
  int height_;
  int width_;

#ifdef EVA_ENABLE
  qrb::image::resize::EvaUtils utils_;
#endif

  void handle_callback(const qrb_ros::transport::type::Image &image_msg);

  rclcpp::TimerBase::SharedPtr fps_timer_;
  std::chrono::time_point<std::chrono::steady_clock> time_resize_start_;
  std::chrono::time_point<std::chrono::steady_clock> time_resize_end_;
  std::chrono::time_point<std::chrono::steady_clock> time_node_start_;
  std::chrono::time_point<std::chrono::steady_clock> time_node_end_;
  uint32_t frame_cnt_;
  uint64_t resize_latency_;
  uint64_t total_latency_;
  bool calculate_enable_;
  bool calculate_start_flag_;
  void calculate_fps_and_latency();
};

} // namespace qrb_ros::resize

#endif // QRB_ROS_IMAGE_RESIZE__RESIZE_HPP_
