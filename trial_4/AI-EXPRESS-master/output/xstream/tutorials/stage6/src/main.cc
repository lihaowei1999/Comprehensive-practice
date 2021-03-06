/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: main.cc
 * @Brief: 
 * @Author: ruoting.ding 
 * @Date: 2020-04-17 16:30:22 
 * @Last Modified by: qingpeng.liu
 * @Last Modified time: 2020-11-02 17:52:09
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "method/callback.h"
#include "hobotxstream/xstream_config.h"
#include "hobotxsdk/xstream_error.h"
#include "hobotxsdk/xstream_sdk.h"
#include "method/bbox.h"
#include "hobotxstream/profiler.h"

int main(int argc, char const *argv[]) {
  using Stage6Async::Callback;
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  int num = 10;
  flow->SetConfig("config_file", "./config/filter.json");
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_name", "flow");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->SetConfig("profiler_fps_interval", "8");
  flow->SetConfig("profiler_time_interval", "100");

  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->Init();
  std::cout << "========Init Finish==============" << std::endl;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1),
      "BBoxFilter_1");
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1),
      "BBoxFilter_2");

  float x1{0};   // BBox(框)的左上角横坐标
  float y1{20};  // BBox(框)的左上角纵坐标
  float x2{0};   // BBox(框)的右上角横坐标
  float y2{50};  // BBox(框)的右上角纵坐标

  for (int i = 0; i < num; i++) {
    x2 = i;
    std::shared_ptr<xstream::BBox> bbox = std::make_shared<xstream::BBox>(
      x1, y1, x2, y2);
    bbox->type_ = "BBox";
    std::shared_ptr<BaseDataVector> data = std::make_shared<BaseDataVector>();
    data->datas_.push_back(BaseDataPtr(bbox));
    data->name_ = "head_box";

    InputDataPtr inputdata = std::make_shared<InputData>();
    inputdata->datas_.push_back(BaseDataPtr(data));

    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  delete flow;

  return 0;
}
