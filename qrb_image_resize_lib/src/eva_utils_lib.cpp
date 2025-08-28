// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_image_resize_lib/eva_utils_lib.hpp"

define ALIGN(x, y)(((x) + (y)-1) & (~((y)-1)))

    namespace qrb::image::resize
{
  static int32_t session_user_data = 0x22222222;

  static void eva_session_callback(
      evaSession /*hSession*/, evaEvent eEvent, void * psession_user_data)
  {
    std::cout << "Session call back" << std::endl;
    if (EVA_EVFATAL == eEvent) {
      std::cerr << "EVA_EVFATAL :Unrecoverable EVA Hardware Error Event" << std::endl;
      return;
    }
    if (*((int32_t *)psession_user_data) == session_user_data) {
      std::cout << "Session User Data matched" << std::endl;
    } else {
      std::cerr << "Session User Data not matched" << std::endl;
    }
  }

  EvaUtils::EvaUtils() : session_(NULL) {}

  EvaUtils::~EvaUtils()
  {
    if (handler_ != nullptr) {
      delete[] eva_config_list_.pConfigs;
      evaStopSession(session_);
      if (type_ == Eva_type::Resize) {
        evaDeInitScaledown(*handler_);
      }
    }
    evaDeleteSession(session_);
  }

  bool EvaUtils::initialized()
  {
    if (session_ != NULL) {
      std::cout << "Session handle has initialized" << std::endl;
      return true;
    }
    auto eva_img_convert_cb = eva_session_callback;
    evaSession session = evaCreateSession(eva_img_convert_cb, (void *)&session_user_data);
    if (session != NULL) {
      session_ = session;
      std::cout << "Session init" << std::endl;
      return true;
    }
    return false;
  }

  int EvaUtils::image_resize(const int32_t & input_fd, const uint32_t & input_width,
      const uint32_t input_height, int32_t & output_fd, const uint32_t & output_width,
      const uint32_t & output_height, const std::string & input_color_format,
      const int interpolation)
  {
    if (session_ == NULL) {
      std::cerr << "Session is null, need call initialized() before this function" << std::endl;
      return -1;
    }

    evaImage input_image;
    evaImage output_image;

    qrb::image::resize::eva_image_param(
        input_width, input_height, input_color_format, &input_image);
    int dst_num_bytes = input_image.sImageInfo.nTotalSize;

    int ret =
        qrb::image::resize::eva_fd_convert_mem(input_fd, dst_num_bytes, &(input_image.pBuffer));
    if (ret < 0) {
      std::cerr << "eva fd convert image failed" << std::endl;
      return -1;
    }

    qrb::image::resize::eva_image_param(
        output_width, output_height, input_color_format, &output_image);

    dst_num_bytes = output_image.sImageInfo.nTotalSize;

    evaStatus status = qrb::image::resize::eva_mem_alloc(
        dst_num_bytes, "/dev/dma_heap/system", EVA_MEM_NON_SECURE, &(output_image.pBuffer));

    if (EVA_SUCCESS != status) {
      std::cerr << "evaMemAlloc failed for pOutput" << std::endl;
      return -1;
    }
    memset(output_image.pBuffer->pAddress, 0, dst_num_bytes);

    if (eva_downscale_height_config_ != input_height &&
        eva_downscale_width_config_ != input_width) {
      if (handler_ != nullptr && type_ == Eva_type::Resize) {
        delete[] eva_config_list_.pConfigs;
        evaStopSession(session_);
        evaDeInitScaledown(*handler_);
      }
      eva_config_list_.nConfigs = 8;
      eva_config_list_.pConfigs = new evaConfig[8];
      evaScaledownQueryConfigIndices(evaScaledownConfigStrings, &eva_config_list_);
      eva_config_list_.pConfigs[0].uValue.u32 = 240;            // actualfps
      eva_config_list_.pConfigs[1].uValue.u32 = 240;            // operationfps
      eva_config_list_.pConfigs[2].uValue.u32 = input_width;    // src width
      eva_config_list_.pConfigs[3].uValue.u32 = input_height;   // src height
      eva_config_list_.pConfigs[4].uValue.u32 = output_width;   // dst width
      eva_config_list_.pConfigs[5].uValue.u32 = output_height;  // dst height
      evaColorFormat src_color_format;
      if ((nv12_.compare(input_color_format)) == 0) {
        src_color_format = EVA_COLORFORMAT_NV12;
      } else if ((mono8_.compare(input_color_format)) == 0) {
        src_color_format = EVA_COLORFORMAT_GRAY_8BIT;
      } else {
        std::cerr << "color format not support, only support mono8, nv12" << std::endl;
        return -1;
      }
      eva_config_list_.pConfigs[6].uValue.ptr = (void *)&src_color_format;
      eva_config_list_.pConfigs[7].uValue.ptr = (void *)&src_color_format;

      evaHandle init_handle = evaInitScaledown(session_, &eva_config_list_, NULL, NULL);
      if (init_handle == NULL) {
        std::cerr << "evaInitScaledown Failed : Init_Handle is NULL" << std::endl;
        return -1;
      }

      status = evaStartSession(session_);
      if (status != EVA_SUCCESS) {
        std::cerr << "start scale session failed" << std::endl;
        return -1;
      }
      handler_ = std::make_shared<evaHandle>(init_handle);
      eva_downscale_height_config_ = input_height;
      eva_downscale_width_config_ = input_width;
      type_ = Eva_type::Resize;
    }

    uint32_t n_scale_down_Interpolation = EVA_SCALEDOWN_BILINEAR;
    if (interpolation == 1) {
      n_scale_down_Interpolation = EVA_SCALEDOWN_BICUBIC;
    }

    evaConfigList config;
    config.nConfigs = 1;
    config.pConfigs = new evaConfig[1];
    config.pConfigs[0].nIndex = 8;
    config.pConfigs[0].eType = EVA_PTR;
    config.pConfigs[0].uValue.ptr = new evaScaledownInterpolation;
    *(evaScaledownInterpolation *)(config.pConfigs[0].uValue.ptr) =
        (evaScaledownInterpolation)n_scale_down_Interpolation;

    status = evaScaledown_Sync(*handler_, &input_image, &output_image, &config);
    if (EVA_SUCCESS != status) {
      std::cerr << "evaScaleDown failed" << std::endl;
      return -1;
    }
    qrb::image::resize::eva_mem_free(input_image.pBuffer);
    output_fd = output_image.pBuffer->nFD;
    qrb::image::resize::eva_mem_free(output_image.pBuffer);
    delete[] config.pConfigs;
    return 0;
  }

}  // namespace qrb::image::resize
