/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: smartplugin impl based on xstream.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */

#include "smartplugin_box/smartplugin.h"

#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xstream_sdk.h"
#include "horizon/vision/util.h"
#include "horizon/vision_type/vision_error.h"
#include "horizon/vision_type/vision_type.hpp"
#include "smartplugin_box/convert.h"
#include "smartplugin_box/displayinfo.h"
#include "smartplugin_box/runtime_monitor.h"
#include "smartplugin_box/smart_config.h"
#include "video_box_common.h"
#include "videoprocessor.h"
#include "votmodule.h"
#include "xproto/message/pluginflow/flowmsg.h"
#include "xproto/message/pluginflow/msg_registry.h"
#include "xproto/plugin/xpluginasync.h"
#include "xproto_msgtype/protobuf/x2.pb.h"
#include "xproto_msgtype/protobuf/x3.pb.h"
#include "xproto_msgtype/vioplugin_data.h"
#include "xwareplugin/waremessage.h"

namespace horizon {
namespace vision {
namespace xproto {
namespace smartplugin_multiplebox {

using horizon::vision::xproto::XPluginAsync;
using horizon::vision::xproto::XProtoMessage;
using horizon::vision::xproto::XProtoMessagePtr;
using horizon::vision::xproto::wareplugin::WarePluginRecogMessage;

using horizon::vision::xproto::basic_msgtype::VioMessage;
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XStreamImageFramePtr = xstream::XStreamData<ImageFramePtr>;

using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

using horizon::iot::smartplugin_multiplebox::Convertor;

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)

SmartPlugin::SmartPlugin(const std::string &smart_config_file) {
  smart_config_file_ = smart_config_file;

  LOGI << "smart config file:" << smart_config_file_;
}

void SmartPlugin::ParseConfig() {
  xstream_workflow_cfg_file_ =
      config_->GetSTDStringValue("xstream_workflow_file");
  xstream_workflow_cfg_pic_file_ =
      config_->GetSTDStringValue("xstream_workflow_file_pic");
  xstream_workflow_cfg_feature_file_ =
      config_->GetSTDStringValue("xstream_workflow_file_feature");
  enable_profile_ = config_->GetBoolValue("enable_profile");
  enable_recog_ = config_->GetBoolValue("enable_recog");
  profile_log_file_ = config_->GetSTDStringValue("profile_log_path");
  if (config_->HasMember("enable_result_to_json")) {
    result_to_json = config_->GetBoolValue("enable_result_to_json");
  }

  if (config_->HasMember("box_face_thr")) {
    smart_vo_cfg_.box_face_thr = config_->GetFloatValue("box_face_thr");
  }
  if (config_->HasMember("box_head_thr")) {
    smart_vo_cfg_.box_head_thr = config_->GetFloatValue("box_head_thr");
  }
  if (config_->HasMember("box_body_thr")) {
    smart_vo_cfg_.box_body_thr = config_->GetFloatValue("box_body_thr");
  }
  if (config_->HasMember("lmk_thr")) {
    smart_vo_cfg_.lmk_thr = config_->GetFloatValue("lmk_thr");
  }
  if (config_->HasMember("kps_thr")) {
    smart_vo_cfg_.kps_thr = config_->GetFloatValue("kps_thr");
  }
  if (config_->HasMember("box_veh_thr")) {
    smart_vo_cfg_.box_veh_thr = config_->GetFloatValue("box_veh_thr");
  }
  if (config_->HasMember("plot_fps")) {
    smart_vo_cfg_.plot_fps = config_->GetBoolValue("plot_fps");
  }

  rtsp_config_file_ = config_->GetSTDStringValue("rtsp_config_file");
  display_config_file_ = config_->GetSTDStringValue("display_config_file");
  run_smart_ = config_->GetBoolValue("run_smart");
  LOGI << "xstream_workflow_file:" << xstream_workflow_cfg_file_;
  LOGI << "enable_profile:" << enable_profile_
       << ", profile_log_path:" << profile_log_file_;
}

int SmartPlugin::Init() {
  Json::Value cfg_jv;
  std::ifstream infile(smart_config_file_);
  infile >> cfg_jv;
  config_.reset(new JsonConfigWrapper(cfg_jv));
  ParseConfig();
  monitor_.reset(new RuntimeMonitor());
  GetDisplayConfigFromFile(display_config_file_);
  GetRtspConfigFromFile(rtsp_config_file_);
  LOGD << "get channel_num from file is:" << channel_num_;
  VideoProcessor::GetInstance().Init(channel_num_, display_mode_, smart_vo_cfg_,
                                     encode_smart_, draw_smart_,
                                     running_venc_1080p_, running_venc_720p_,
                                     running_vot_, draw_real_time_video_);
  if (!run_smart_) {
    LOGW << "video box config to not run smart, just decode and display!!!";
    return 0;
  }
  // init for xstream sdk
  LOGI << "smart plugin init";
  sdk_.resize(channel_num_);

  for (int i = 0; i < channel_num_; i++) {
    sdk_[i].reset(xstream::XStreamSDK::CreateSDK());
    sdk_[i]->SetConfig("config_file", xstream_workflow_cfg_file_);
    if (sdk_[i]->Init() != 0) {
      LOGE << "smart plugin init failed!!!";
      return -1;
    }

    sdk_[i]->SetCallback(
        std::bind(&SmartPlugin::OnCallback, this, std::placeholders::_1));
  }

  RegisterMsg(TYPE_IMAGE_MESSAGE,
              std::bind(&SmartPlugin::Feed, this, std::placeholders::_1));
  if (enable_recog_ == true) {
    if (xstream_workflow_cfg_pic_file_ != "") {
      pic_sdk_.reset(xstream::XStreamSDK::CreateSDK());
      pic_sdk_->SetConfig("config_file", xstream_workflow_cfg_pic_file_);
      if (pic_sdk_->Init() != 0) {
        LOGE << "smart plugin init failed!!!";
        return -1;
      }

      pic_sdk_->SetCallback(
          std::bind(&SmartPlugin::OnCallbackPic, this, std::placeholders::_1));
    }
    if (xstream_workflow_cfg_feature_file_ != "") {
      feature_sdk_.reset(xstream::XStreamSDK::CreateSDK());
      feature_sdk_->SetConfig("config_file",
          xstream_workflow_cfg_feature_file_);
      if (feature_sdk_->Init() != 0) {
        LOGE << "smart plugin init failed!!!";
        return -1;
      }
      feature_sdk_->SetCallback(
       std::bind(&SmartPlugin::OnCallbackFeature, this, std::placeholders::_1));
    }
    RegisterMsg(TYPE_FACE_PIC_IMAGE_MESSAGE,
                std::bind(&SmartPlugin::FeedPic, this, std::placeholders::_1));
    RegisterMsg(TYPE_RECOG_MESSAGE,
        std::bind(&SmartPlugin::FeedRecog, this, std::placeholders::_1));
  }

  return XPluginAsync::Init();
}

int SmartPlugin::Feed(XProtoMessagePtr msg) {
  // parse valid frame from msg
  LOGD << "smart plugin got one msg";
  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  xstream::InputDataPtr input = Convertor::ConvertInput(valid_frame.get());
  SmartInput *input_wrapper = new SmartInput();
  if (input_wrapper == NULL) {
    LOGE << "new smart input ptr fail, return error!";
    return -1;
  }
  input_wrapper->frame_info = valid_frame;
  input_wrapper->context = input_wrapper;

  int channel_id = input_wrapper->frame_info->channel_;
  monitor_->PushFrame(input_wrapper);
  monitor_->FrameStatistic(channel_id);

  if (channel_id >= channel_num_) {
    LOGE << "there is no channel num:" << channel_id << "for feed stream!!!";
    return -1;
  }

  input->context_ = (const void *)((uintptr_t)channel_id);
  if (sdk_[channel_id]->AsyncPredict(input) != 0) {
    return -1;
  }

  LOGD << "Feed one task to xtream workflow, channel_id " << channel_id
       << " frame_id " << valid_frame->image_[0]->frame_id;

  return 0;
}


int SmartPlugin::FeedPic(XProtoMessagePtr msg) {
  // parse valid frame from msg
  LOGD << "smart plugin got one msg";
  std::cout << "FeedPic,smart plugin got one msg" << std::endl;
  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  xstream::InputDataPtr input = Convertor::ConvertInput(valid_frame.get());
  if (!pic_sdk_ || (pic_sdk_->AsyncPredict(input) != 0)) {
    return -1;
  }
  return 0;
}

int SmartPlugin::FeedRecog(XProtoMessagePtr msg) {
  auto recog_msg = std::static_pointer_cast<WarePluginRecogMessage>(msg);
  auto cach = recog_msg->GetMessageData();
  std::lock_guard<std::mutex> lk(cache_mtx_);
  recog_cache_[cach->ch_id][cach->track_id] = cach;
  return 0;
}

int SmartPlugin::Start() {
  VideoProcessor::GetInstance().Start();
  if (!run_smart_) {
    LOGW << "video box config to not run smart, just decode and display!!!";
    return 0;
  }

  LOGI << "SmartPlugin Start";
  root.clear();
  running_ = true;
  return 0;
}

int SmartPlugin::Stop() {
  VideoProcessor::GetInstance().Stop();
  if (!run_smart_) {
    LOGW << "video box config to not run smart, just decode and display!!!";
    return 0;
  }
  running_ = false;
  if (result_to_json) {
    remove("smart_data.json");
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream outputFileStream("smart_data.json");
    writer->write(root, &outputFileStream);
  }
  LOGI << "SmartPlugin Stop";
  return 0;
}

void SmartPlugin::OnCallback(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGD << "smart plugin got one smart result";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";
  XStreamImageFramePtr *rgb_image = nullptr;
  int channel_id = static_cast<int>((uintptr_t)xstream_out->context_);
  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    } else if (output->name_ == "snap_list") {
      auto snap_v =
          std::dynamic_pointer_cast<xstream::BaseDataVector>(output);
      if ((feature_sdk_) && (snap_v->datas_.size() > 0)) {
        LOGE << "snap_v size:" << snap_v->datas_.size();
        xstream::InputDataPtr input = std::make_shared<xstream::InputData>();
        input->datas_.emplace_back(output);
        input->context_ = (const void *)((uintptr_t)channel_id);
        feature_sdk_->AsyncPredict(input);
      }
    }
  }
  HOBOT_CHECK(rgb_image);
  if (draw_real_time_video_ && draw_smart_) {
    HorizonVisionSmartFrame *smart_frame = nullptr;
    HorizonVisionAllocSmartFrame(&smart_frame);
    Convertor::ConvertOutputToSmartFrame(xstream_out, smart_frame);
    VideoProcessor::GetInstance().Input(channel_id, smart_frame);
  }

  LOGD << "OnCallback channel id " << channel_id << " frame_id "
       << rgb_image->value->frame_id;
  if ((!draw_real_time_video_ && running_vot_ &&
       !smart_vo_cfg_.transition_support) ||
      (running_venc_1080p_ && encode_smart_)) {
    int layer =
        DisplayInfo::computePymLayer(display_mode_, channel_num_, channel_id);
    auto video_data = std::make_shared<horizon::vision::VideoData>();
    HorizonVisionAllocSmartFrame(&video_data->smart_frame);
    // HorizonVisionCopySmartFrame(smart_frame, video_data->smart_frame);
    Convertor::ConvertOutputToSmartFrame(xstream_out, video_data->smart_frame);
    uint32_t width_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .width;
    uint32_t height_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .height;
    char *y_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .y_vaddr);
    char *uv_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .c_vaddr);
    video_data->channel = channel_id;
    video_data->width = width_tmp;
    video_data->height = height_tmp;
    video_data->data_len = width_tmp * height_tmp * 3 / 2;
    video_data->buffer =
        static_cast<char *>(malloc(width_tmp * height_tmp * 3 / 2));
    for (uint32_t i = 0; i < height_tmp; ++i) {
      memcpy(video_data->buffer + i * width_tmp, y_addr + i * width_tmp,
             width_tmp);
    }

    for (uint32_t i = 0; i < (height_tmp / 2); ++i) {
      memcpy(video_data->buffer + (i + height_tmp) * width_tmp,
             uv_addr + i * width_tmp, width_tmp);
    }
    video_data->recog_cache = recog_cache_[channel_id];
    VideoProcessor::GetInstance().Input(video_data);
  }

  if (running_venc_720p_ && encode_smart_) {
    int layer = DisplayInfo::computePymLayer(display_mode_, channel_num_,
                                             channel_id, true);
    auto video_data = std::make_shared<horizon::vision::VideoData>();
    HorizonVisionAllocSmartFrame(&video_data->smart_frame);
    Convertor::ConvertOutputToSmartFrame(xstream_out, video_data->smart_frame);
    uint32_t width_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .width;
    uint32_t height_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .height;
    video_data->channel = channel_id;
    video_data->width = width_tmp;
    video_data->height = height_tmp;
    video_data->data_len = width_tmp * height_tmp * 3 / 2;
    video_data->buffer =
        static_cast<char *>(malloc(width_tmp * height_tmp * 3 / 2));
    char *y_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .y_vaddr);
    char *uv_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .c_vaddr);
    for (uint32_t i = 0; i < height_tmp; ++i) {
      memcpy(video_data->buffer + i * width_tmp, y_addr + i * width_tmp,
             width_tmp);
    }

    for (uint32_t i = 0; i < (height_tmp / 2); ++i) {
      memcpy(video_data->buffer + (i + height_tmp) * width_tmp,
             uv_addr + i * width_tmp, width_tmp);
    }
    VideoProcessor::GetInstance().Input(video_data, true);
  }

  if (smart_vo_cfg_.transition_support && !draw_real_time_video_) {
    int layer = 0;
    auto video_data = std::make_shared<horizon::vision::VideoData>();
    uint32_t width_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .width;
    uint32_t height_tmp =
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .height;
    video_data->channel = channel_id;
    video_data->width = width_tmp;
    video_data->height = height_tmp;
    video_data->data_len = width_tmp * height_tmp * 3 / 2;
    video_data->y_virtual_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .y_vaddr);
    video_data->uv_virtual_addr = reinterpret_cast<char *>(
        ((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
            ->down_scale[layer]
            .c_vaddr);
    VideoProcessor::GetInstance().Input(video_data, false, true);
  }

  auto input = monitor_->PopFrame(rgb_image->value->frame_id, channel_id);
  delete static_cast<SmartInput *>(input.context);
}


void SmartPlugin::OnCallbackPic(xstream::OutputDataPtr xstream_out) {
  LOGI << "OnCallbackPic smart plugin got one smart result";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";
  XStreamImageFramePtr *rgb_image = nullptr;
  for (const auto &output : xstream_out->datas_) {
    LOGI << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    }
    HOBOT_CHECK(rgb_image);
    if (output->name_ == "face_feature") {
      std::shared_ptr<FeatureFrameMessage> feat_msg =
             std::make_shared<FeatureFrameMessage>();
      Convertor::ConvertOutput(xstream_out, *feat_msg.get());
      if (feat_msg->error_code_ == 0) {
        auto feature_msg = std::make_shared<SmartFeatureMessage>(feat_msg);
        feature_msg->SetMessageType("Pic_Face");
        PushMsg(feature_msg);
      }
    }
  }
}


void SmartPlugin::OnCallbackFeature(xstream::OutputDataPtr xstream_out) {
  int channel_id = static_cast<int>((uintptr_t)xstream_out->context_);
  for (const auto &output : xstream_out->datas_) {
    if (output->name_ == "face_feature") {
      auto features_v =
              std::dynamic_pointer_cast<xstream::BaseDataVector>(output);
      if (features_v->datas_.size() > 0) {
        LOGD << "OnCallback   face_feature";
        std::shared_ptr<FeatureFrameMessage> feat_msg =
               std::make_shared<FeatureFrameMessage>();
        Convertor::ConvertOutput(xstream_out, *feat_msg.get());
        feat_msg->ch_id_ = channel_id;
        if (feat_msg->error_code_ == 0) {
          auto feature_msg = std::make_shared<SmartFeatureMessage>(feat_msg);
          feature_msg->SetMessageType("Real_Face");
          PushMsg(feature_msg);
        }
      }
    }
  }
}

void SmartPlugin::GetRtspConfigFromFile(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOGE << "Open config file " << path << " failed";
    return;
  }
  Json::Value config_;
  ifs >> config_;
  ifs.close();

  auto value_js = config_["channel_num"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: channel_num";
  }
  channel_num_ = value_js.asInt();
}

void SmartPlugin::GetDisplayConfigFromFile(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOGE << "Open config file " << path << " failed";
    return;
  }
  Json::Value config_;
  ifs >> config_;
  ifs.close();

  running_vot_ = config_["vo"]["enable"].asBool();
  display_mode_ = config_["vo"]["display_mode"].asInt();
  smart_vo_cfg_.transition_support =
      config_["vo"]["transition_support"].asBool();
  draw_smart_ = config_["vo"]["draw_smart"].asBool();
  draw_real_time_video_ = config_["vo"]["draw_real_time_video"].asBool();

  running_venc_1080p_ = config_["rtsp"]["stream_1080p"].asBool();
  running_venc_720p_ = config_["rtsp"]["stream_720p"].asBool();
  encode_smart_ = config_["rtsp"]["encode_smart_info"].asBool();
}

}  // namespace smartplugin_multiplebox
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
