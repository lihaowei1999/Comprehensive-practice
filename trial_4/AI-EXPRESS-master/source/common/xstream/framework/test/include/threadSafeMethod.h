/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file OrderTestMethod.h
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 * @date 2019/11/25
 */

#ifndef TEST_INCLUDE_THREADSAFEMETHOD_H_
#define TEST_INCLUDE_THREADSAFEMETHOD_H_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/data_types/orderdata.h"

namespace xstream {

class SafeTestThread : public Method {
 public:
  int Init(const std::string &config_file_path) override {
    shared_safe_value = 0;
    return 0;
}

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<xstream::InputParamPtr> &param) override {
  std::cout << "SafeTestThread::DoProcess" << std::endl;
  shared_safe_value.fetch_add(1);
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    auto &in_batch_i = input[i];
    auto &out_batch_i = output[i];
    out_batch_i.resize(in_batch_i.size());
    std::cout << "input size: " << in_batch_i.size() << std::endl;
    // 只支持n个输入，输入格式是BBox的数组
    for (size_t j = 0; j < in_batch_i.size(); ++j) {
      if (in_batch_i[j]->state_ == DataState::INVALID) {
        std::cout << "input slot " << j << " is invalid" << std::endl;
        continue;
      }
      auto in_datas = std::static_pointer_cast<BaseDataVector>(in_batch_i[j]);
      auto out_datas = std::make_shared<BaseDataVector>();
      out_batch_i[j] = std::static_pointer_cast<BaseData>(out_datas);
      for (auto &in_data : in_datas->datas_) {
        auto safeData = std::static_pointer_cast<xstream::OrderData>(in_data);
        safeData->sequence_id = shared_safe_value;
        std::cout << "!!!!!safemethod safeData sequence_id "
        << safeData->sequence_id << std::endl;
        out_datas->datas_.push_back(in_data);
      }
    }
  }

    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_int_distribution<> dis(1, 100);

    int duration = dis(engine);
    std::cout << "this thread sleep " << duration << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));

    return output;
  }

  void Finalize() override {}

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  InputParamPtr GetParameter() const override { return InputParamPtr(); }

  std::string GetVersion() const override { return "0.0.0"; }

  MethodInfo GetMethodInfo() override {
    MethodInfo orderMethod = MethodInfo();
    orderMethod.is_thread_safe_ = true;
    orderMethod.is_need_reorder = true;

    return orderMethod;
  }

 private:
  std::atomic<int> shared_safe_value;
};
}  // namespace xstream
#endif  // TEST_INCLUDE_THREADSAFEMETHOD_H_

