/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: RaiseHandEvent.cpp
 * @Brief: definition of the RaiseHandEvent
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-05-25 14:27:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-05-25 16:17:08
 */

#include "BehaviorMethod/RaiseHandEvent.h"
#include <vector>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision_type/vision_type.hpp"

namespace xstream {

int RaiseHandEvent::Init(const Json::Value &config) {
  BehaviorEvent::Init(config);
  if (config.isMember("arm_slope")) {
    tan_threshold_ = config["arm_slope"].asFloat();
    HOBOT_CHECK(tan_threshold_ > 0);
  }
  if (config.isMember("skeleton_score_thres")) {
    skeleton_score_thres_ = config["skeleton_score_thres"].asFloat();
    HOBOT_CHECK(skeleton_score_thres_ > 0);
  }
  return 0;
}

bool RaiseHandEvent::IsEvent(hobot::vision::Landmarks kps) {
  //system("echo 1 > ../../webservice/html/modules/data.html \n");
  //system("echo 1 > ../../data.html \n");
  //system("echo 1 > data_n.html");
  //system("echo 1 > /userdata/data.html");
  //system("echo 'test' > /userdata/final/deploy/data.html");
  if (kps.values[9].score < skeleton_score_thres_ ||
      kps.values[10].score < skeleton_score_thres_ ||
      kps.values[6].score < skeleton_score_thres_ ||
      kps.values[5].score < skeleton_score_thres_ ||
      kps.values[3].score < skeleton_score_thres_ ||
      kps.values[4].score < skeleton_score_thres_) {
    system("echo 'no arm' > /userdata/final/deploy/webservice/html/modules/data.html");
    return false;
  }

  // 任意手臂举起即可
  bool is_raise = false;
  // right arm (Observed object left arm)
  {
    auto a = kps.values[9].y - kps.values[5].y;
    auto b = kps.values[9].x - kps.values[5].x;
    auto c = kps.values[9].y - kps.values[3].y;
    if (a < 0 && c < 0) {
      if (CalculateSlope(a, b)>= tan_threshold_) {
        is_raise = true;
        LOGI << "right arm raise";
        system("echo 'right arm' > /userdata/final/deploy/webservice/html/modules/data.html");
      }
    }
  }
  //if (is_raise) return true;

  // left arm
  {
    auto a = kps.values[10].y - kps.values[6].y;
    auto b = kps.values[10].x - kps.values[6].x;
    auto c = kps.values[10].y - kps.values[4].y;

    if (a < 0 && c < 0) {
      if (CalculateSlope(a, b)>= tan_threshold_) {
        if(is_raise == true)
        {
          system("echo 'both arm' > /userdata/final/deploy/webservice/html/modules/data.html");
        }else{
          system("echo 'left arm' > /userdata/final/deploy/webservice/html/modules/data.html");
        }
        is_raise = true;
        LOGI << "left arm raise";
      }
    }
  }
  if (!is_raise) system("echo 'no raise' > /userdata/final/deploy/webservice/html/modules/data.html");
  return is_raise;
}

}  // namespace xstream
