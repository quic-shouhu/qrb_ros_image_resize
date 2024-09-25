// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_IMAGE_RESIZE__EVA_UTILS_LIB_HPP_
#define QRB_IMAGE_RESIZE__EVA_UTILS_LIB_HPP_

#include "qrb_image_resize_lib/eva_common_lib.hpp"
#include <memory>
#include <string>

namespace qrb::image::resize
{

enum Eva_type {
  None,
  Resize,
};

class EvaUtils
{
public:
  explicit EvaUtils();

  ~EvaUtils();

  int image_resize(const int32_t &input_fd, const uint32_t &input_width, const uint32_t input_height,
                    int32_t &output_fd, const uint32_t &output_width, const uint32_t &output_height,
                    const std::string &input_color_format, const int interpolation = 0);

  bool initialized();

private:
  evaConfigList eva_config_list_;
  uint32_t eva_downscale_height_config_;
  uint32_t eva_downscale_width_config_;

  evaSession session_;
  const std::string nv12_ = "nv12";
  const std::string mono8_ = "mono8";
  Eva_type type_;
  std::shared_ptr<evaHandle> handler_;
};

} // namespace qrb::image::resize

#endif // QRB_IMAGE_RESIZE__EVA_UTILS_LIB_HPP_
