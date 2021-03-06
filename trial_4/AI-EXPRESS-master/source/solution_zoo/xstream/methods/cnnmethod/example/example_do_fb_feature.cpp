//
// Created by yaoyao.sun on 2019-04-29.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include "CNNMethod/CNNMethod.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/image_tools.h"
#include "hobotxsdk/xstream_sdk.h"
#include "horizon/vision_type/vision_type.hpp"
#include "opencv2/opencv.hpp"

typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;

static void Usage() {
  std::cout << "./example do_fb_feature " <<
  "xstream_cfg_file img_lmk_list out_file\n";
}

using xstream::CNNMethod;

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::XStreamData;

using hobot::vision::BBox;
using hobot::vision::CVImageFrame;
using hobot::vision::Landmarks;
using hobot::vision::Point;
using hobot::vision::Points;
using hobot::vision::SnapshotInfo;

template <typename DType>
struct LmkSnapInfo : SnapshotInfo<DType> {
  Points PointsToSnap(const Points &in) { return in; }
};

void PrintFaceFeature(const std::vector<xstream::BaseDataPtr> &result,
                      std::ostream &output) {
  auto face_feature =
      std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  auto target_size = face_feature->datas_.size();
  for (size_t target_idx = 0; target_idx < target_size; target_idx++) {
    auto features = std::static_pointer_cast<xstream::BaseDataVector>(
        face_feature->datas_[target_idx]);
    for (size_t snap_idx = 0; snap_idx < features->datas_.size(); snap_idx++) {
      auto feature = std::static_pointer_cast<xstream::XStreamData<
                     hobot::vision::Feature>>(features->datas_[target_idx]);
      static int feature_size = 128;
      if (feature->state_ != xstream::DataState::VALID) {
        for (int i = 0; i < feature_size; i++) {
          output << " " << std::fixed << std::setprecision(5) << -1.0f;
        }
      } else {
        for (int i = 0; i < feature_size; i++) {
          output << " " << std::fixed << std::setprecision(5)
                 << feature->value.values[i];
        }
      }
    }
  }
  output << std::endl;
}

int DoFbFeature(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return -1;
  }
  std::string cfg_file = argv[1];
  std::string img_list = argv[2];
  std::string output_file = argv[3];

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->Init();

  std::ifstream ifs_img_list(img_list);
  if (!ifs_img_list.is_open()) {
    LOGD << "open image list file failed." << std::endl;
    return -1;
  }

  std::ofstream output(output_file, std::ios::out);

  std::string gt_data;
  std::string input_image;
  Landmarks landmark;
  landmark.values.resize(5);
  while (getline(ifs_img_list, gt_data)) {
    std::istringstream gt(gt_data);
    gt >> input_image;
    for (auto &point : landmark.values) {
      gt >> point.x >> point.y;
      LOGD << "x: " << point.x << " y: " << point.y;
    }

    auto img_bgr = cv::imread(input_image);
    int width = img_bgr.cols;
    int height = img_bgr.rows;
    LOGD << "origin image size, width: " << img_bgr.cols
         << ", height: " << img_bgr.rows << std::endl;

    cv::Mat yuv_420(height * 3 / 2, width, CV_8UC1);
    cv::cvtColor(img_bgr, yuv_420, CV_BGR2YUV_I420);
    uint8_t *output_data = nullptr;
    int output_size, output_1_stride, output_2_stride;
    HobotXStreamConvertImage(yuv_420.data,
                          height * width * 3 / 2,
                          width, height,
                          width, width / 2,
                          IMAGE_TOOLS_RAW_YUV_I420,
                          IMAGE_TOOLS_RAW_YUV_NV12,
                          &output_data, &output_size,
                          &output_1_stride,
                          &output_2_stride);

    cv::Mat img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
    memcpy(img_nv12.data, output_data, output_size);
    HobotXStreamFreeImage(output_data);

    auto face_img = std::make_shared<CVImageFrame>();

    face_img->img = img_nv12;
    face_img->pixel_format =
        HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawNV12;

    auto xstream_landmarks = std::make_shared<XStreamData<Landmarks>>();
    xstream_landmarks->value = landmark;

    auto snap = std::make_shared<XStreamData<std::shared_ptr<
                LmkSnapInfo<BaseDataPtr>>>>();
    auto snap_shot_info = std::make_shared<LmkSnapInfo<BaseDataPtr>>();
    snap->value = snap_shot_info;
    snap_shot_info->userdata.resize(2);
    snap_shot_info->userdata[1] = xstream_landmarks;
    snap_shot_info->snap = face_img;

    auto p_persons = std::make_shared<BaseDataVector>();
    auto p_one_person = std::make_shared<BaseDataVector>();
    p_persons->datas_.push_back(p_one_person);
    p_persons->name_ = "snap_list";
    p_one_person->datas_.push_back(snap);

    xstream::InputDataPtr inputdata(new xstream::InputData());
    inputdata->datas_.push_back(
        std::static_pointer_cast<xstream::BaseData>(p_persons));
    auto out = flow->SyncPredict(inputdata);
    std::vector<std::string> dirs;
    xstream::split_string(input_image, dirs, "/");
    std::string track_id = dirs.size() >= 2 ? dirs[dirs.size() - 2] : dirs[0];
    output << track_id;
    PrintFaceFeature(out->datas_, output);
  }
  delete flow;
  return 0;
}
