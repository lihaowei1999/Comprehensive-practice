/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: BehaviorMethod.h
 * @Brief: declaration of the BehaviorMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-05-25 14:18:28
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-05-25 16:01:54
 */

#ifndef BEHAVIORMETHOD_BEHAVIORMETHOD_H_
#define BEHAVIORMETHOD_BEHAVIORMETHOD_H_

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include "hobotxstream/simple_method.h"
#include "BehaviorMethod/BehaviorEvent.h"
#include "json/json.h"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxstream/profiler.h"
#include "hobotlog/hobotlog.hpp"
#include "bpu_predict/bpu_predict_extension.h"
#include "hobotxsdk/xstream_data.h"

#ifdef X3
#include "./bpu_predict_x3.h"
#endif

namespace xstream {
class BehaviorMethod : public SimpleMethod {
 public:
  BehaviorMethod() {}
  virtual ~BehaviorMethod() {}

  virtual int Init(const std::string &cfg_path);
  virtual void Finalize();

  virtual std::vector<BaseDataPtr>
  DoProcess(const std::vector<BaseDataPtr> &input,
            const xstream::InputParamPtr &param);
  virtual int UpdateParameter(xstream::InputParamPtr ptr);
  virtual InputParamPtr GetParameter() const;
  virtual std::string GetVersion() const;

 private:
  Json::Value config_;
  std::shared_ptr<BehaviorEvent> behavior_event_;

};
}  // namespace xstream

#endif  // BEHAVIORMETHOD_BEHAVIORMETHOD_H_
