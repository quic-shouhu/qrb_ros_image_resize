// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_IMAGE_RESIZE__EVA_COMMON_LIB_HPP_
#define QRB_IMAGE_RESIZE__EVA_COMMON_LIB_HPP_

#include "evaMem.h"
#include "evaScale.h"
#include "evaSession.h"
#include "evaTypes.h"
#include "evaUtils.h"
#include "evaWarp.h"

#include <BufferAllocator/BufferAllocator.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace qrb::image::resize {
void eva_mem_free(evaMem *p_mem, bool need_close = false) {
  struct dma_buf_sync buf_sync;
  buf_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
  int32_t rc = ioctl(p_mem->nFD, DMA_BUF_IOCTL_SYNC, &buf_sync);
  if (rc) {
    std::cerr << "[EVA COMMON] eva mem free: ioctl failed" << std::endl;
  }
  munmap(p_mem->pAddress, p_mem->nSize);
  if (need_close) {
    close(p_mem->nFD);
  }
  p_mem->nFD = -1;
  p_mem->pAddress = NULL;
  delete p_mem;
}

// release the eva image object.
void eva_image_free(evaImage *image) { eva_mem_free(image->pBuffer); }

// convert the fd info into eva image 's pBuffer.
int eva_fd_convert_mem(const int32_t &fd, const uint32_t &size, evaMem **mem) {
  evaMem *p_mem = new evaMem();
  if (p_mem == NULL) {
    std::cerr << "[EVA COMMON] fd convert mem: create Mem failed" << std::endl;
    return -1;
  }
  p_mem->eType = EVA_MEM_NON_SECURE;
  p_mem->nSize = size;
  p_mem->nFD = fd;
  uint8_t *tmp = (uint8_t *)mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
  if (tmp == MAP_FAILED) {
    std::cerr << "[EVA COMMON] fd convert mem: mmap failed" << std::endl;
    delete p_mem;
    return -1;
  }
  p_mem->pAddress = tmp;
  struct dma_buf_sync buf_sync;
  buf_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
  int32_t rc = ioctl(fd, DMA_BUF_IOCTL_SYNC, &buf_sync);
  if (rc) {
    std::cerr << "[EVA COMMON] fd convert mem: ioctl failed" << std::endl;
    delete p_mem;
    return -1;
  }
  *mem = p_mem;
  return 0;
}

// use the unalignment width and height to storage in eva image 's sImageInfo.
int eva_image_param(const uint32_t &width, const uint32_t &height,
                    const std::string &color_format, evaImage *image) {
  image->sImageInfo.nWidth = width;
  image->sImageInfo.nHeight = height;
  evaImageInfo p_query_src_image_info;
  evaColorFormat eva_color_format = (evaColorFormat)0;
  std::string nv12 = "nv12";
  std::string mono8 = "mono8";
  if ((nv12.compare(color_format)) == 0) {
    eva_color_format = EVA_COLORFORMAT_NV12;
  } else if ((mono8.compare(color_format)) == 0) {
    eva_color_format = EVA_COLORFORMAT_GRAY_8BIT;
  }
  evaStatus queryimage_status = evaQueryImageInfo(
      eva_color_format, width, height, &p_query_src_image_info);
  if (queryimage_status != EVA_SUCCESS) {
    std::cerr << "[EVA COMMON] query image status failed" << std::endl;
    return -1;
  }
  for (uint32_t index = 0; index < p_query_src_image_info.nPlane; index++) {
    image->sImageInfo.nWidthStride[index] =
        p_query_src_image_info.nWidthStride[index];
    image->sImageInfo.nAlignedSize[index] =
        p_query_src_image_info.nAlignedSize[index];
  }
  image->sImageInfo.nPlane = p_query_src_image_info.nPlane;
  if (p_query_src_image_info.nTotalSize % 4096 == 0) {
    image->sImageInfo.nTotalSize = p_query_src_image_info.nTotalSize;
  } else {
    image->sImageInfo.nTotalSize =
        (p_query_src_image_info.nTotalSize / 4096 + 1) * 4096;
  }
  image->sImageInfo.eFormat = eva_color_format;
  return 0;
}

// alloc the buffer in dma_buf, need call eva_image_param to get the total size
// before this api.
evaStatus eva_mem_alloc(uint32_t size, const std::string &heap_name,
                        evaMemSecureType e_secure_type, evaMem **mem) {
  evaMem *p_mem = new evaMem();
  if (p_mem == NULL) {
    std::cerr << "[EVA COMMON] eva image alloc: create Mem failed" << std::endl;
    return EVA_EFAIL;
  }
  BufferAllocator allocator;
  uint32_t alignment = 0x100000;
  int32_t m_fd = allocator.Alloc(heap_name, size, 0, alignment);
  if (m_fd < 0) {
    std::cerr << "[EVA COMMON] eva image alloc: alloc failed" << std::endl;
    delete p_mem;
    return EVA_EFAIL;
  }
  uint8_t *dst =
      (uint8_t *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
  if (dst == MAP_FAILED) {
    std::cerr << "[EVA COMMON] eva image alloc: mmap failed" << std::endl;
    delete p_mem;
    return EVA_EFAIL;
  }
  struct dma_buf_sync buf_sync;
  buf_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
  int32_t rc = ioctl(m_fd, DMA_BUF_IOCTL_SYNC, &buf_sync);
  if (rc) {
    std::cerr << "[EVA COMMON] eva image alloc: ioctl failed" << std::endl;
    delete p_mem;
    return EVA_EFAIL;
  }
  p_mem->eType = e_secure_type;
  p_mem->nSize = size;
  p_mem->nFD = m_fd;
  p_mem->pAddress = dst;
  *mem = p_mem;
  return EVA_SUCCESS;
}

// alloc the image in dma_buf, need call eva_image_param to get the total size
// before this api.
evaStatus eva_image_alloc(const std::string &heap_name,
                          evaMemSecureType e_secure_type, evaImage *image) {
  return eva_mem_alloc(image->sImageInfo.nTotalSize, heap_name, e_secure_type,
                       &(image->pBuffer));
}

}; // namespace qrb::image::resize

#endif // QRB_IMAGE_RESIZE__EVA_COMMON_LIB_H
