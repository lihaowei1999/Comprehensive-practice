/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethod.cpp
 * @Brief: definition of the CNNMethod
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:17:08
 */

#include "CNNMethod/CNNMethod.h"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "./plat_cnn.h"
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/PostPredictor/PostPredictor.h"
#include "CNNMethod/PostPredictor/PostPredictorFactory.h"
#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/Predictor/PredictorFactory.h"
#include "CNNMethod/util/CNNMethodConfig.h"
#include "CNNMethod/util/CNNMethodData.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision_type/vision_type.hpp"

namespace xstream {

int32_t CNNMethod::Init(const std::string &cfg_path) {
  LOGI << "CNNMethod Init";
  std::ifstream infile(cfg_path.c_str());
  HOBOT_CHECK(infile.good()) << "CNNMethod error config file path:" << cfg_path;

  std::stringstream buffer;
  buffer << infile.rdbuf();
  config_.reset(new CNNMethodConfig(buffer.str()));
  config_->SetSTDStringValue("parent_path", get_parent_path(cfg_path));

  std::string input_type = config_->GetSTDStringValue("in_msg_type");

  auto iter = g_input_type_map.find(input_type);
  HOBOT_CHECK(iter != g_input_type_map.end())
      << "in_msg_type unknown: " << input_type;
  predictor_.reset(PredictorFactory::GetPredictor(iter->second));

  std::string post_fn = config_->GetSTDStringValue("post_fn");
  auto fn_iter = g_post_fun_map.find(post_fn);
  HOBOT_CHECK(fn_iter != g_post_fun_map.end()) << "post_fn unknown:" << post_fn;
  post_predict_.reset(PostPredictorFactory::GetPostPredictor(fn_iter->second));

  predictor_->Init(config_);
  post_predict_->Init(config_);
  return 0;
}

MethodInfo CNNMethod::GetMethodInfo() {
  MethodInfo method_info = Method::GetMethodInfo();
  if (predictor_) {
    method_info.is_need_reorder = predictor_->IsNeedReorder();
  }
  return method_info;
}

void CNNMethod::Finalize() {}

std::vector<BaseDataPtr> CNNMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  HOBOT_CHECK(input.size() > 0);
  CNNMethodRunData run_data;
  run_data.input = &input;
  run_data.param = &param;

  predictor_->Do(&run_data);
  post_predict_->Do(&run_data);
  return run_data.output;
}

int CNNMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    CNNMethodConfig cf(content);
    UpdateParams(cf.config, config_->config);
    predictor_->UpdateParam(config_);
    post_predict_->UpdateParam(config_);
    return 0;
  } else {
    HOBOT_CHECK(0) << "only support json format config";
    return -1;
  }
}

InputParamPtr CNNMethod::GetParameter() const {
  return std::static_pointer_cast<xstream::InputParam>(config_);
}

std::string CNNMethod::GetVersion() const { return predictor_->GetVersion(); }

}  // namespace xstream
