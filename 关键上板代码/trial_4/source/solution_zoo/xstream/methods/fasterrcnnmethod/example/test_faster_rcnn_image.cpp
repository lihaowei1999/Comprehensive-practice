/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <assert.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <fstream>
#include <memory>
#include <vector>

#include "./yuv_utils.h"
#include "FasterRCNNMethod/dump.h"

#include "horizon/vision_type/vision_type.hpp"

#include "hobotxsdk/xstream_sdk.h"
#include "opencv2/opencv.hpp"

#include "FasterRCNNMethod.h"
#include "bpu_predict/bpu_predict.h"



using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::XStreamData;
using xstream::InputParamPtr;

using xstream::FasterRCNNMethod;
using hobot::vision::ImageFrame;
using hobot::vision::CVImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example faster_rcnn_image "
               "config_file img_list_file\n";
}

int TestFasterRCNNImage(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  std::string img_list = argv[2];

  FasterRCNNMethod faster_rcnn_method;
  std::string config_file = argv[1];
  faster_rcnn_method.Init(config_file);
  auto faster_rcnn_param =
    std::make_shared<xstream::FasterRCNNParam>("FasterRCNNMethod");
  faster_rcnn_param->max_face_count = 0;
  faster_rcnn_method.UpdateParameter(faster_rcnn_param);

  std::cout << "fasterrcnn init success." << std::endl;

  std::ifstream ifs(img_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }
  static int64_t frame_id = 0;
  std::string input_image;
  while (getline(ifs, input_image)) {
    auto img_bgr = cv::imread(input_image);
    std::cout << "origin image size, width: " << img_bgr.cols
              << ", height: " << img_bgr.rows << std::endl;
    auto cv_image_frame_ptr = std::make_shared<CVImageFrame>();

    cv_image_frame_ptr->img = img_bgr;
    cv_image_frame_ptr->pixel_format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
    auto xstream_img =
      std::make_shared<XStreamData<std::shared_ptr<ImageFrame>>>();
    xstream_img->value = cv_image_frame_ptr;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(xstream_img);
    std::vector<BaseDataPtr> xstream_output =
        faster_rcnn_method.DoProcess(input, param);
    std::cout << "predict success: " << frame_id++ << std::endl;
  }
  faster_rcnn_method.Finalize();

  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
