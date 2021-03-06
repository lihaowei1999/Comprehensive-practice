/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VENCMODULE_H_
#define INCLUDE_VENCMODULE_H_

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "hb_comm_venc.h"
#include "hobot_vision/blocking_queue.hpp"
#include "hobotxsdk/xstream_data.h"
#include "video_box_common.h"
#include "visualplugin/horizonserver_api.h"

namespace horizon {
namespace vision {

struct VencModuleInfo {
  uint32_t width;
  uint32_t height;
  uint32_t type;
  uint32_t bits;
};

struct VencBuffer {
  char *mmz_vaddr = nullptr;
  uint64_t mmz_paddr;
  uint32_t mmz_size;
  uint32_t mmz_flag = 0;
  std::mutex mmz_mtx;
};

class VencModule {
 public:
  VencModule();
  ~VencModule();
  int Init(uint32_t chn_id, const VencModuleInfo *module_info,
           const int channel_num, const smart_vo_cfg_t &smart_vo_cfg,
           const bool encode_smart = false, const int display_mode = 0);
  int Start();
  int Input(std::shared_ptr<VideoData> video_data, const bool copy = false);

  int Output(void **data);
  int OutputBufferFree(void *data);
  int Stop();
  int DeInit();
  int Process();

 private:
  int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
                      int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt);
  int HandleData();
  int EncodeData(std::shared_ptr<VideoData> video_data);

 private:
  int InitVPS();
  int ResizePic();
  int DeInitVPS();

  static std::once_flag flag_;
  uint32_t chn_id_;
  uint32_t timeout_;
  bool init_ = false;

  VencModuleInfo venc_info_;
  VencBuffer buffers_;

  hb_vio_buffer_t *buffer_1080p_ = nullptr;
  hb_vio_buffer_t *gdc_buf = nullptr;

  FILE *outfile_;
  int pipe_fd_;

  bool process_running_ = false;
  std::shared_ptr<std::thread> process_thread_;

  uint32_t display_mode_ = 0;
  uint32_t channel_num_ = 0;
  bool encode_720p_ = false;
  bool encode_smart_ = false;
  smart_vo_cfg_t vo_plot_cfg_;
  std::shared_ptr<std::thread> encode_thread_;
  hobot::vision::BlockingQueue<std::shared_ptr<VideoData>> in_queue_;
  uint32_t in_queue_len_max_ = 40;
};

}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_VencModule_H_
