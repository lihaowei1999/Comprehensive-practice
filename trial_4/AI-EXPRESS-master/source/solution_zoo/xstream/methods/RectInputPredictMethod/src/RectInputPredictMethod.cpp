/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: RectInputPredictMethod.cpp
 * @Brief: implementation of RectInputPredictMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu Dec 24 2020 11:27:59
 */

#include "RectInputPredictMethod/RectInputPredictMethod.h"
#include <fstream>
#include <string>
#include <vector>
#include "DnnAsyncData.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/image_tools.h"
#include "hobotxstream/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifdef X3
#include "./bpu_predict_x3.h"
#endif

using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;
using hobot::vision::BBox;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace xstream {

int RectInputPredictMethod::Init(const std::string &cfg_path) {
  LOGD << "RectInputPredictMethod Init";
  DnnPredictMethod::Init(cfg_path);

  // model_path, dnn_run_with_roi is got in Init() of parent class
  std::string norm_method = config_["norm_method"].isString()
                                ? config_["norm_method"].asString()
                                : "norm_by_nothing";
  auto norm_iter = g_norm_method_map.find(norm_method);
  HOBOT_CHECK(norm_iter != g_norm_method_map.end()) << "unknown norm method: "
                                                    << norm_method;
  norm_params_.norm_method = norm_iter->second;
  norm_params_.expand_scale = config_["expand_scale"].isDouble()
                                  ? config_["expand_scale"].asFloat()
                                  : 1.0f;
  norm_params_.aspect_ratio = config_["aspect_ratio"].isDouble()
                                  ? config_["aspect_ratio"].asFloat()
                                  : 1.0f;

  std::string filter_method = config_["filter_method"].isString()
                                  ? config_["filter_method"].asString()
                                  : "no_filter";
  auto filter_iter = g_filter_method_map.find(filter_method);
  HOBOT_CHECK(filter_iter != g_filter_method_map.end())
      << "unknown filter method: " << filter_method;
  filter_method_ = filter_iter->second;

  // TODO(shiyu.fu): default data type, usage?
  input_data_type_ = config_["filter_method"].isString()
                         ? config_["filter_method"].asString()
                         : "";

  max_handle_num_ = config_["max_handle_num"].isInt()
                        ? config_["max_handle_num"].asUInt()
                        : -1;

  return 0;
}

int RectInputPredictMethod::PrepareInputData(
    const std::vector<BaseDataPtr> &input, const InputParamPtr param,
    hobot::vision::PymImageFrame &pyramid, std::vector<BPU_BBOX> &input_bbox,
    std::vector<int> &valid_box, std::vector<BPU_TENSOR_S> &output_tensors) {
  LOGD << "RectInputPredictMethod PrepareInputData";

  auto rois = std::static_pointer_cast<BaseDataVector>(input[0]);
  auto xstream_pyramid =
      std::static_pointer_cast<XStreamData<ImageFramePtr>>(input[1]);
  auto pyramid_frame =
      std::static_pointer_cast<PymImageFrame>(xstream_pyramid->value);
  pyramid = *pyramid_frame;

  uint32_t box_num = rois->datas_.size();
  valid_box.resize(box_num, 1);

  uint32_t handle_num =
      max_handle_num_ < 0 ? box_num : std::min(max_handle_num_, box_num);
  LOGD << "vision bbox num : " << box_num;
  for (size_t idx = 0; idx < box_num; ++idx) {
    auto &roi = rois->datas_[idx];
    auto p_roi = std::static_pointer_cast<XStreamData<BBox>>(roi);
    auto p_normed_roi = std::make_shared<XStreamData<BBox>>();
    if (p_roi->state_ != xstream::DataState::VALID ||
        norm_params_.norm_method == NormMethod::BPU_MODEL_NORM_BY_NOTHING) {
      p_normed_roi->value = p_roi->value;
    } else {
      BBox *normed_box = &(p_normed_roi->value);
      NormalizeRoi(&p_roi->value, normed_box, norm_params_, pyramid.Width(),
                   pyramid.Height(), filter_method_);
      LOGD << "norm roi norm_type:"
           << static_cast<int>(norm_params_.norm_method)
           << " expand_scale:" << norm_params_.expand_scale
           << " aspect_ratio:" << norm_params_.aspect_ratio
           << "  from:" << p_roi->value.x1 << ", " << p_roi->value.y1 << ", "
           << p_roi->value.x2 << ", " << p_roi->value.y2
           << "  to:" << p_normed_roi->value.x1 << ", "
           << p_normed_roi->value.y1 << ", " << p_normed_roi->value.x2 << ", "
           << p_normed_roi->value.y2;
    }

    if (p_roi->state_ != xstream::DataState::VALID || idx >= handle_num) {
      valid_box[idx] = 0;
    } else {
      input_bbox.push_back(
          BPU_BBOX{p_normed_roi->value.x1, p_normed_roi->value.y1,
                   p_normed_roi->value.x2, p_normed_roi->value.y2,
                   p_normed_roi->value.score, 0, true});
    }
  }

  if (input_bbox.size() <= 0) {
    LOGD << "no box to bpu";
    return -1;
  }
  LOGD << "BPU_BBOX num: " << input_bbox.size();

  // alloc output tensor
  int ret = AllocOutputTensor(output_tensors, input_bbox.size());
  if (ret != 0) {
    LOGE << "error alloc output tensor, code: " << ret;
    FreeTensor(output_tensors);
    return -1;
  }

  return 0;
}

}  // namespace xstream
