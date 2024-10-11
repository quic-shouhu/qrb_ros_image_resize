// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <qrb_ros_image_resize/resize.hpp>

using namespace qrb_ros::transport::image_utils;
namespace qrb_ros::resize
{
ResizeNode::ResizeNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ResizeNode", options)
{
  RCLCPP_INFO(this->get_logger(), " == Received image start eva resize == ");

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_sub_ = this->create_subscription<qrb_ros::transport::type::Image>("image_raw", 10,
      std::bind(&ResizeNode::handle_callback, this, std::placeholders::_1), sub_options);

  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_pub_ =
      this->create_publisher<qrb_ros::transport::type::Image>("image_resize", 10, pub_options);

  interpolation_ = this->declare_parameter("interpolation", 0);
  use_scale_ = this->declare_parameter("use_scale", false);
  scale_height_ = this->declare_parameter("scale_height", 1.0);
  scale_width_ = this->declare_parameter("scale_width", 1.0);
  height_ = this->declare_parameter("height", -1);
  width_ = this->declare_parameter("width", -1);

  calculate_enable_ = this->declare_parameter("calculate_enable", false);
  calculate_start_flag_ = false;
  total_latency_ = 0;
  resize_latency_ = 0;
  frame_cnt_ = 0;
}

void ResizeNode::handle_callback(const qrb_ros::transport::type::Image & image_msg)
{
  RCLCPP_DEBUG(this->get_logger(), " == Received image run eva resize == ");

  if (calculate_enable_ && !calculate_start_flag_) {
    calculate_start_flag_ = true;
    RCLCPP_INFO(this->get_logger(), " -- calculate fps and latency start -- ");
    fps_timer_ = this->create_wall_timer(std::chrono::seconds(CALCULATE_TIME),
        std::bind(&ResizeNode::calculate_fps_and_latency, this));
  }
  if (calculate_enable_) {
    time_node_start_ = std::chrono::steady_clock::now();
  }

  auto out_msg = std::make_unique<qrb_ros::transport::type::Image>();
  out_msg->height = use_scale_ ? image_msg.height * scale_height_ : height_;
  out_msg->width = use_scale_ ? image_msg.width * scale_width_ : width_;

  uint32_t aligned_width = align_width(image_msg.width);
  uint32_t aligned_height = align_height(image_msg.height);
  uint32_t dst_aligned_width = align_width(out_msg->width);
  uint32_t dst_aligned_height = align_height(out_msg->height);

  // Ensure the image src and dst size is valid
  if (image_msg.height > MAX_HEIGHT || image_msg.width > MAX_WIDTH || image_msg.height <= 0 ||
      image_msg.width <= 0) {
    RCLCPP_ERROR(this->get_logger(), "The src image size is not supported");
    return;
  }
  if (out_msg->height < MIN_DST || out_msg->width < MIN_DST || out_msg->height > image_msg.height ||
      out_msg->width > image_msg.width) {
    RCLCPP_ERROR(this->get_logger(), "The dst image size is not supported");
    return;
  }
  if (use_scale_) {
    if (scale_height_ < LIMIT_SCALE || scale_width_ < LIMIT_SCALE || scale_height_ > 1 ||
        scale_width_ > 1) {
      RCLCPP_ERROR(this->get_logger(), "The dst image scale_width/height is not supported");
      return;
    }
  } else {
    if (out_msg->height * 8 < image_msg.height || out_msg->width * 8 < image_msg.width) {
      RCLCPP_ERROR(this->get_logger(), "The dst image width/height scale is not supported");
      return;
    }
  }

  // Get the image encoding and Ensure the image encoding is valid
  std::string encoding = image_msg.encoding;
  if (encoding != "nv12") {
    RCLCPP_ERROR(this->get_logger(), "Image encoding is not supported: %s", encoding.c_str());
    return;
  }

  // Ensure the image interpolation is valid
  if (interpolation_ != 0 && interpolation_ != 1) {
    RCLCPP_ERROR(this->get_logger(), "Image interpolation is not supported: %d", interpolation_);
    return;
  }

  // Initialize eva
  if (!utils_.initialized()) {
    RCLCPP_ERROR(this->get_logger(), "eva utils init failed");
    return;
  }

  // eva resize image
  int32_t out_fd = -1;

  if (calculate_enable_) {
    time_resize_start_ = std::chrono::steady_clock::now();
  }
  int status = utils_.image_resize(image_msg.dmabuf->fd(), image_msg.width, image_msg.height,
      out_fd, out_msg->width, out_msg->height, encoding, interpolation_);
  if (calculate_enable_) {
    time_resize_end_ = std::chrono::steady_clock::now();
  }

  if (status != 0) {
    RCLCPP_ERROR(this->get_logger(), "Resize failed with status: %d, output fd: %d, encoding: %s",
        status, out_fd, encoding.c_str());
    return;
  }

  // alloc dmabuf
  auto dst_size = get_image_align_size(out_msg->width, out_msg->height, encoding);
  auto dmabuf = std::make_shared<lib_mem_dmabuf::DmaBuffer>(out_fd, dst_size);
  if (!dmabuf) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate dmabuf");
    return;
  }
  out_msg->dmabuf = dmabuf;
  out_msg->header = image_msg.header;
  out_msg->encoding = encoding;

  // publish image
  image_pub_->publish(std::move(out_msg));
  RCLCPP_DEBUG(this->get_logger(), "Published resized image.");

  if (calculate_enable_) {
    time_node_end_ = std::chrono::steady_clock::now();
    auto time_resize =
        std::chrono::duration_cast<std::chrono::microseconds>(time_resize_end_ - time_resize_start_)
            .count();
    auto time_node =
        std::chrono::duration_cast<std::chrono::microseconds>(time_node_end_ - time_node_start_)
            .count();
    resize_latency_ += time_resize;
    total_latency_ += time_node;
    frame_cnt_++;
  }
}

void ResizeNode::calculate_fps_and_latency()
{
  auto current_time = std::chrono::steady_clock::now();
  double fps = static_cast<double>(frame_cnt_) / static_cast<double>(CALCULATE_TIME);
  if (total_latency_ <= static_cast<double>(CALCULATE_TIME) * 1000000.0 && total_latency_ > 0) {
    RCLCPP_INFO(this->get_logger(), "Current FPS: %.2f", fps);
    RCLCPP_INFO(this->get_logger(), "Average resize Latency: %.2f ms",
        (static_cast<double>(resize_latency_) / frame_cnt_) / 1000);
    RCLCPP_INFO(this->get_logger(), "Average node Latency: %.2f ms",
        (static_cast<double>(total_latency_) / frame_cnt_) / 1000);
  }

  // Reset counters and start time
  frame_cnt_ = 0;
  resize_latency_ = 0;
  total_latency_ = 0;
}

}  // namespace qrb_ros::resize

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::resize::ResizeNode)
