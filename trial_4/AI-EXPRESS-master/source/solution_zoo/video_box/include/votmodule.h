/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VOTMODULE_H_
#define INCLUDE_VOTMODULE_H_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <map>

#include "hb_vio_interface.h"
#include "hobot_vision/blocking_queue.hpp"
#include "hobotxsdk/xstream_data.h"
#include "mediapipemanager/basicmediamoudle.h"
#include "opencv2/opencv.hpp"
#include "smartplugin_box/character_font.h"
#include "smartplugin_box/traffic_info.h"
#include "video_box_common.h"
#include "xstream/vision_type/include/horizon/vision_type/vision_type.h"

using horizon::vision::xproto::smartplugin::VehicleInfo;

namespace horizon {
namespace vision {

class VotModule {
 public:
  VotModule();
  ~VotModule();
  int Init(uint32_t group_id, const smart_vo_cfg_t &smart_vo_cfg);
  int Start();
  int Input(std::shared_ptr<VideoData> video_data,
            const bool transition = false);
  int Input(const int channel, HorizonVisionSmartFrame *smart_frame);

  int Output(void **data);
  int OutputBufferFree(void *data);
  int Stop();
  int DeInit();

  void SetDisplayMode(const int display_mode) { display_mode_ = display_mode; }
  void SetChannelNum(const int channel_num) { channel_num_ = channel_num; }
  void SetDrawSmart(const bool draw_smart) { draw_smart_ = draw_smart; }
  void SetDrawRealTimeVideo(const bool flag) { draw_real_time_video_ = flag; }

 private:
  uint32_t group_id_;
  uint32_t timeout_;
  uint32_t image_width_;
  uint32_t image_height_;
  uint32_t display_mode_;
  uint32_t channel_num_;
  bool draw_smart_ = false;
  bool draw_real_time_video_ = true;

  static uint64_t frame_id_;
  static std::mutex frame_id_mtx_;

  uint32_t frame_interval_;
  uint32_t frame_transition_;
  bool display_chn_hide_;
  char *buffer_;
  char *buffer1_;
  smart_vo_cfg_t vo_plot_cfg_;

  typedef struct logo_img_cache_s {
    int top_image_width_;
    int top_image_height_;
    cv::Mat top_yuv_mat_;
    cv::Mat top_bgr_mat_;
    cv::Mat top_bgr_mat_left_;
    cv::Mat top_bgr_mat_right_;
    cv::Mat top_bgr_mat_mid_;

    int bottom_image_width_;
    int bottom_image_height_;
    cv::Mat bottom_yuv_mat_;
    cv::Mat bottom_bgr_mat_;
    cv::Mat bottom_bgr_mat_left_;
    cv::Mat bottom_bgr_mat_right_;
    cv::Mat bottom_bgr_mat_mid_;
  } logo_img_cache_t;
  logo_img_cache_t logo_img_cache_;
  int ParseLogoImg(const std::string &file_name_top,
                   const std::string &file_name_bottom, int pad_width = 1920,
                   int pad_height = 1080);
  int ParseBottomLogoImg(const std::string &file_name_bottom_left,
                         const std::string &file_name_bottom_rigth);
  void padding_logo(char *buf, int pad_width = 1920, int pad_height = 1080);

  int PlotFont(char *y, const char *font_buf, int x0, int y0,
               int bg_width = 1920, int bg_height = 1080);

  // position 0:top, 1:bottom
  // left_right 0:left, 1:right
  int Drawlogo(const cv::Mat &logo, cv::Mat *bgr, int position,
               int left_right = 0);
  void DrawLogo(cv::Mat *bgr, const int channel);

  void DataToBuffer(const uint32_t src_width, const uint32_t src_height,
                    cv::Mat &bgr_mat, char *buf, int channel, bool &resize);
  void bgr_to_nv12(uint8_t *bgr, uint8_t *buf, const uint32_t width,
                   const uint32_t height);
  void DataToBuffer(char *buf, std::shared_ptr<VideoData> video_data);
  int HandleData();
  int Transition(std::shared_ptr<VideoData> video_data);

 private:
  bool start_ = false;
  bool init_ = false;
  bool running_ = false;

  hobot::vision::BlockingQueue<std::shared_ptr<VideoData>> in_queue_;
  uint32_t in_queue_len_max_ = 40;
  // std::mutex cache_mtx_;
  std::thread plot_task_;

  std::vector<std::shared_ptr<std::thread>> plot_threads_;
  std::mutex buffer_mutex_;

  std::map<int, hobot::vision::BlockingQueue<HorizonVisionSmartFrame*>>
      smart_frame_list_;
};

}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_VOTMODULE_H_