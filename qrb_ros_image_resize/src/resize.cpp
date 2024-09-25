// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <qrb_ros_image_resize/resize.hpp>

using namespace qrb_ros::transport::image_utils;

namespace qrb_ros::resize
{
ResizeNode::ResizeNode(const rclcpp::NodeOptions &options) : rclcpp::Node("ResizeNode", options)
{
  RCLCPP_INFO(this->get_logger()," == Received image run opencv resize == ");

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_sub_ = this->create_subscription<qrb_ros::transport::type::Image>("image_raw",
        10, std::bind(&ResizeNode::handle_callback, this, std::placeholders::_1), sub_options);

  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_pub_ = this->create_publisher<qrb_ros::transport::type::Image>("image_resize",
        10, pub_options);

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

void ResizeNode::handle_callback(const qrb_ros::transport::type::Image &image_msg)
{
  RCLCPP_DEBUG(this->get_logger()," == Received image run opencv resize == ");

  if(calculate_enable_ && !calculate_start_flag_) {
    calculate_start_flag_ = true;
    RCLCPP_INFO(this->get_logger()," -- calculate fps and latency start -- ");
    fps_timer_ = this->create_wall_timer(std::chrono::seconds(CALCULATE_TIME),
                            std::bind(&ResizeNode::calculate_fps_and_latency, this));
  }
  if(calculate_enable_) {
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
  if (image_msg.height > MAX_HEIGHT || image_msg.width > MAX_WIDTH ||
      image_msg.height <= 0 || image_msg.width <= 0) {
    RCLCPP_ERROR(this->get_logger(), "The src image size is not supported");
    return;
  }
  if (out_msg->height < MIN_DST || out_msg->width < MIN_DST ||
      out_msg->height > image_msg.height || out_msg->width > image_msg.width) {
    RCLCPP_ERROR(this->get_logger(), "The dst image size is not supported");
    return;
  }
  if (use_scale_) {
    if (scale_height_ < LIMIT_SCALE || scale_width_ < LIMIT_SCALE ||
        scale_height_ > 1 || scale_width_ > 1) {
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
  cv::InterpolationFlags interp_flag = static_cast<cv::InterpolationFlags>(interpolation_ + 2);

  // read image from dmabuf
  auto src_size = image_msg.width * image_msg.height * 1.5;
  auto data = std::make_unique<char[]>(src_size);
  auto step = get_image_stride(image_msg.width,image_msg.encoding);
  read_image_from_dmabuf(image_msg.dmabuf, data.get(), image_msg.width, image_msg.height,
                                                                step, image_msg.encoding, true);

  // Convert image nv12 to rgb
  cv_bridge::CvImagePtr rgb(new cv_bridge::CvImage);
  cv::Mat y_plane_s(image_msg.height, image_msg.width, CV_8UC1, data.get());
  cv::Mat uv_plane_s(image_msg.height / 2, image_msg.width / 2, CV_8UC2,
                                              data.get() + image_msg.width * image_msg.height);
  cv::cvtColorTwoPlane(y_plane_s, uv_plane_s, rgb->image, cv::COLOR_YUV2RGB_NV12);

  // opencv resize image
  cv::Mat resized_image;

  if(calculate_enable_) {
    time_resize_start_ = std::chrono::steady_clock::now();
  }
  if (use_scale_) {
    cv::resize(rgb->image, resized_image, cv::Size(), scale_width_, scale_height_, interp_flag);
  } else {
    cv::resize(rgb->image, resized_image,
                                  cv::Size(out_msg->width, out_msg->height), 0, 0, interp_flag);
  }
  if(calculate_enable_) {
    time_resize_end_ = std::chrono::steady_clock::now();
  }
  RCLCPP_DEBUG(this->get_logger(), "resize done");

  // Convert resized RGB to YUV I420 format
  cv::Mat yuv_image;
  cv::cvtColor(resized_image, yuv_image, cv::COLOR_RGB2YUV_I420);
  uint32_t uv_height = out_msg->height / 2;
  uint32_t uv_width = out_msg->width / 2;
  uint32_t nLenY = out_msg->height * out_msg->width;
  uint32_t nLenU = nLenY / 4;
  // Split Y, U, and V planes
  cv::Mat y_plane_d = yuv_image(cv::Rect(0, 0, out_msg->width, out_msg->height));
  cv::Mat u_plane_d = yuv_image(cv::Rect(0, out_msg->height, uv_width, uv_height));
  cv::Mat v_plane_d = yuv_image(cv::Rect(
                            uv_width, out_msg->height, uv_width, uv_height));
  // Initialize NV12 data array
  std::vector<uchar> nv12_data(out_msg->width * out_msg->height * 3 / 2);
  // Copy Y plane
  for (int32_t i = 0; i < out_msg->height; ++i) {
    memcpy(nv12_data.data() + i * out_msg->width, 
                                        y_plane_d.data + i * out_msg->width, out_msg->width);
  }
  // Copy UV plane
  for (uint32_t j = 0; j < uv_height; j++) {
    for (uint32_t i = 0; i < uv_width; i++) {
      nv12_data[nLenY + j * out_msg->width + 2 * i] = u_plane_d.data[j * uv_width + i];
      nv12_data[nLenY + j * out_msg->width + 2 * i + 1] = v_plane_d.data[j * uv_width + i];
    }
  }

  // Allocate dmabuf
  auto dst_size = get_image_align_size(out_msg->width, out_msg->height, encoding);
  auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(dst_size, "/dev/dma_heap/system");
  if (dmabuf == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer alloc failed");
  }
  step = out_msg->width;
  if (!save_image_to_dmabuf(dmabuf, nv12_data.data(),
                                      out_msg->width, out_msg->height, step, encoding, true)) {
    RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer write failed");
  }
  out_msg->dmabuf = dmabuf;

  // publish image
  out_msg->header = image_msg.header;
  out_msg->encoding = image_msg.encoding;
  image_pub_->publish(std::move(out_msg));
  RCLCPP_DEBUG(this->get_logger(), "Published resized image.");

  if(calculate_enable_) {
    time_node_end_ = std::chrono::steady_clock::now();
    auto time_resize = std::chrono::duration_cast<std::chrono::microseconds>(
                                                          time_resize_end_ - time_resize_start_).count();
    auto time_node = std::chrono::duration_cast<std::chrono::microseconds>(
                                                        time_node_end_ - time_node_start_) .count();
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

} // namespace qrb_ros::resize

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::resize::ResizeNode)
