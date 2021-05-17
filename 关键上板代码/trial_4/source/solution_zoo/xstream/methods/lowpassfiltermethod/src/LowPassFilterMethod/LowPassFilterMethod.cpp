 /*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     LowPassFilter Method
 * @author    ronghui.zhang
 * @email     ronghui.zhang@horizon.ai
 * @version   0.0.0.1
 * @date      2020.11.02
 */

#include "LowPassFilterMethod/LowPassFilterMethod.h"

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xstream_data.h"
#include "horizon/vision_type/vision_error.h"
#include "horizon/vision_type/vision_type.hpp"

namespace xstream {

int LowPassFilterMethod::Init(const std::string &config_file) {
  LOGI << "LowPassFilterMethod::Init config file " << config_file;
  std::ifstream config_ifs(config_file);
  if (!config_ifs.good()) {
    LOGF << "open config file failed.";
    return 0;
  }
  Json::Value config_jv;
  config_ifs >> config_jv;
  if (config_jv.isMember("mincutoff") && config_jv["mincutoff"].asFloat()) {
    mincutoff_ = config_jv["mincutoff"].asFloat();
  }
  if (config_jv.isMember("beta") && config_jv["beta"].asFloat()) {
    beta_ = config_jv["beta"].asFloat();
  }
  if (config_jv.isMember("dcutoff") && config_jv["dcutoff"].asFloat()) {
    dcutoff_ = config_jv["dcutoff"].asFloat();
  }

  if (config_jv.isMember("frequence") && config_jv["frequence"].asFloat()) {
    freq_ = config_jv["frequence"].asInt();
  }

  return 0;
}

std::vector<BaseDataPtr> LowPassFilterMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &params) {
  LOGD << "LowPassFilter::DoProcess: " << input.size();
  std::vector<BaseDataPtr> output;
  RunSingleFrame(input, output);
  return output;
}

void LowPassFilterMethod::RunSingleFrame(
    const std::vector<BaseDataPtr> &frame_input,
    std::vector<BaseDataPtr> &frame_output) {
  LOGD << "LowPassFilterMethod::RunSingleFrame";
  HOBOT_CHECK(frame_input.size() == 3);
  const auto &in_body_box_list =
    std::static_pointer_cast<BaseDataVector>(frame_input[0])->datas_;
  const auto &dispeared_body_box_list =
    std::static_pointer_cast<BaseDataVector>(frame_input[1])->datas_;
  const auto &in_kps =
    std::static_pointer_cast<BaseDataVector>(frame_input[2])->datas_;
  LOGD << "kps size = " << in_kps.size() <<
  " box size " << in_body_box_list.size();
  BaseDataVectorPtr out_kps = std::make_shared<BaseDataVector>();
  BaseDataVectorPtr out_body_box_list = std::make_shared<BaseDataVector>();
  static double timestamp = 0.0;

  for (size_t i = 0; i < in_body_box_list.size(); i++) {
    const auto &box =
      std::static_pointer_cast<XStreamData<BBox>>(in_body_box_list[i])->value;
    auto &lmk = std::static_pointer_cast<
            xstream::XStreamData<hobot::vision::Landmarks>>(in_kps[i])->value;
    if (box.id == -1 || lmk.values.size() == 0) {
      // invalid id or lmk, passthrough box & lmk
      out_kps->datas_.push_back(in_kps[i]);
      out_body_box_list->datas_.push_back(in_body_box_list[i]);
      continue;
    }

    if (id_set_.find(box.id) != id_set_.end()) {
      LOGD << " find box id " << box.id;
      auto new_lmks = std::make_shared<XStreamData<hobot::vision::Landmarks>>();
      auto new_box = std::make_shared<XStreamData<hobot::vision::BBox>>();
      if (lmk.values.size() != kps_filter_pairs_[box.id].filter_vector.size()) {
        LOGF << "lmk size " << lmk.values.size()
        << "filters size " << kps_filter_pairs_[box.id].filter_vector.size();
      }
      for (size_t j = 0; j < lmk.values.size(); j++) {
        double filtered_x = kps_filter_pairs_[box.id].filter_vector[j]->
        filter_x->filter(lmk.values[j].x, timestamp);
        double filtered_y = kps_filter_pairs_[box.id].filter_vector[j]->
        filter_y->filter(lmk.values[j].y, timestamp);
        LOGD << "lmk x = " << lmk.values[j].x << " lmk y = "
        << lmk.values[j].y << " filtered_x " << filtered_x <<
        "filtered_y " << filtered_y;
        hobot::vision::Point new_p(filtered_x, filtered_y);
        new_p.score = lmk.values[j].score;
        new_lmks->value.values.push_back(new_p);
      }
      new_lmks->value.score = lmk.score;
      out_kps->datas_.push_back(new_lmks);

      new_box->value.x1 = box_filter_pairs_[box.id].filter_vector[0]->
      filter_x->filter(box.x1, timestamp);
      new_box->value.y1 = box_filter_pairs_[box.id].filter_vector[0]->
      filter_y->filter(box.y1, timestamp);

      new_box->value.x2 = box_filter_pairs_[box.id].filter_vector[1]->
      filter_x->filter(box.x2, timestamp);
      new_box->value.y2 = box_filter_pairs_[box.id].filter_vector[1]->
      filter_y->filter(box.y2, timestamp);
      new_box->value.score = box.score;
      new_box->value.id = box.id;
      out_body_box_list->datas_.push_back(new_box);
    } else {
      // insert new id
      id_set_.insert(box.id);
      LOGD << "insert box id " << box.id;
      filtersVector kps_filterVector;
      filtersVector box_filterVector;
      LOGD << "insert lmk size " << lmk.values.size();
      for (size_t j = 0; j < lmk.values.size(); j++) {
        std::shared_ptr<pair_filter> filters =
        std::make_shared<pair_filter>(freq_, mincutoff_, beta_, dcutoff_);
        kps_filterVector.filter_vector.push_back(filters);
      }

      for (size_t j = 0; j < 2; j++) {
        std::shared_ptr<pair_filter> filters =
        std::make_shared<pair_filter>(freq_, mincutoff_, beta_, dcutoff_);
        box_filterVector.filter_vector.push_back(filters);
      }

      kps_filter_pairs_.insert(std::pair<
      int, filtersVector>(box.id, kps_filterVector));
      box_filter_pairs_.insert(std::pair<
      int, filtersVector>(box.id, box_filterVector));
      out_kps->datas_.push_back(in_kps[i]);
      out_body_box_list->datas_.push_back(in_body_box_list[i]);
    }
  }

  for (size_t i = 0; i < dispeared_body_box_list.size(); i++) {
    auto id = std::static_pointer_cast<XStreamUint32>(
      dispeared_body_box_list[i])->value;
    std::set<int>::iterator id_iter = id_set_.find(id);
    if (id_iter != id_set_.end()) {
      id_set_.erase(id_iter);
    }
    std::map<int, filtersVector>::iterator kps_iter =
      kps_filter_pairs_.find(id);
    if (kps_iter != kps_filter_pairs_.end()) {
      kps_filter_pairs_.erase(kps_iter);
    }
    std::map<int, filtersVector>::iterator body_iter =
      box_filter_pairs_.find(id);
    if (body_iter != box_filter_pairs_.end()) {
      box_filter_pairs_.erase(body_iter);
    }
  }

  LOGD << "out kps num = " << out_kps->datas_.size() <<
  " box size " << in_body_box_list.size();
  frame_output.emplace_back(out_kps);
  frame_output.emplace_back(out_body_box_list);
  timestamp += 1.0/freq_;
  return;
}

void LowPassFilterMethod::Finalize() {
}

int LowPassFilterMethod::UpdateParameter(InputParamPtr ptr) {
  return 0;
}

InputParamPtr LowPassFilterMethod::GetParameter() const {
  return nullptr;
}
}  // namespace xstream
