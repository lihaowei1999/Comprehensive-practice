/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "./dump_util.h"
#include "./stopwatch.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "FasterRCNNMethod/faster_rcnn_imp.h"
#include "FasterRCNNMethod/yuv_utils.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xstream_sdk.h"
#ifdef X2
#include "./vio_wrapper.h"
#endif
#ifdef X3
#include "./vio_wrapper_global.h"
#endif
#include "json/json.h"
#include "opencv2/opencv.hpp"
#include "horizon/vision_type/vision_type.hpp"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::XStreamData;

using xstream::FasterRCNNMethod;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example fb_fasterrcnn model_name"
               "[face/face_pose_lmk/multitask/vehicle]"
               " image_list need_render[0/1]\n";
}

typedef void (*dump_fn)(cv::Mat *, const std::vector<BaseDataPtr> &,
                        std::string);

static std::map<std::string, dump_fn> model_name2dump_fn = {
    {"face", DumpFace},
    {"face_pose_lmk", DumpFaceLmk},
    {"multitask", DumpMultitask},
    {"vehicle", DumpVehicle}};

int TestFBFasterrcnn(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  FasterRCNNMethod faster_rcnn_method;
  std::string model_name = argv[1];
  std::string image_list = argv[2];
  bool need_render = false;
  if (argc > 3) {
    need_render = atoi(argv[3]);
  }
  std::string model_config_file;
  if (model_name == "face") {
    model_config_file = "./configs/face_config.json";
  } else if (model_name == "face_pose_lmk") {
    model_config_file = "./configs/face_pose_lmk_config.json";
  } else if (model_name == "multitask") {
    model_config_file = "./configs/multitask_config.json";
  } else if (model_name == "vehicle") {
    model_config_file = "./configs/vehicle_config.json";
  } else {
    std::cout << "not support this model: " << model_name << "\n";
    return -1;
  }
  faster_rcnn_method.Init(model_config_file);

  std::ifstream ifs(image_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }
  std::string input_image;
#ifdef X2
  HbVioFbWrapper fb_vio("./configs/vio_onsemi0230_fb.json");
  auto ret = fb_vio.Init();
  HOBOT_CHECK(ret == 0) << "fb vio init failed!!!";
  img_info_t data;

  Stopwatch time_count;
  time_count.Reset();
  while (getline(ifs, input_image)) {
    cv::Mat bgr_img = cv::imread(input_image);
    int width = bgr_img.cols;
    int height = bgr_img.rows;
    cv::Mat img_nv12;
    bgr_to_nv12(bgr_img.data, height, width, img_nv12);
    ret = fb_vio.GetImgInfo(img_nv12.data, width, height, &data);
    HOBOT_CHECK(ret == 0) << "fb vio get image failed!!!";
    // vio_debug::print_info(data);
    std::string::size_type pos2;
    pos2 = input_image.rfind("/");
    if (pos2 != std::string::npos) input_image = input_image.substr(pos2 + 1);

#if 0
    auto &img_info = data1;
    int img_height = img_info.src_img.height;
    int img_witdh = img_info.src_img.step;
    int img_y_len = img_height * img_witdh;
    int img_uv_len = img_height * img_witdh / 2;
    uint8_t *img_ptr = static_cast<uint8_t*>(malloc(img_y_len+img_uv_len));
    memcpy(img_ptr, img_info.src_img.y_vaddr, img_y_len);
    memcpy(img_ptr+img_y_len, img_info.src_img.c_vaddr, img_uv_len);
    cv::Mat bgr_mat;
    nv12_to_bgr(img_ptr, img_height, img_witdh, bgr_mat);
    free(img_ptr);
    for (int k = 0; k < 5; ++k) {
      int ds_img_height = img_info.down_scale[4*k].height;
      int ds_img_witdh = img_info.down_scale[4*k].step;
      std::cout << "ds_img" << std::to_string(k) <<": height: "
                << ds_img_height << " width: " << ds_img_witdh << std::endl;
      int ds_img_y_len = ds_img_height * ds_img_witdh;
      int ds_img_uv_len = ds_img_height * ds_img_witdh / 2;
      uint8_t *ds_img_ptr = static_cast<uint8_t*>(malloc(
          ds_img_y_len + ds_img_uv_len));
      memcpy(ds_img_ptr, img_info.down_scale[4*k].y_vaddr, ds_img_y_len);
      memcpy(ds_img_ptr + ds_img_y_len, img_info.down_scale[4*k].c_vaddr,
          ds_img_uv_len);
      cv::Mat ds_bgr_mat;
      nv12_to_bgr(ds_img_ptr, ds_img_height, ds_img_witdh, ds_bgr_mat);
      free(ds_img_ptr);
      cv::imwrite("ds_pyramid_img" + std::to_string(k) + ".jpg", ds_bgr_mat);
    }
#endif

    auto py_image_frame_ptr = std::make_shared<PymImageFrame>();
    py_image_frame_ptr->img = data;

    auto xstream_pyramid =
        std::make_shared<XStreamData<std::shared_ptr<ImageFrame>>>();
    xstream_pyramid->value = py_image_frame_ptr;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(xstream_pyramid);

    time_count.Start();
    auto xstream_output = faster_rcnn_method.DoProcess(input, param);
    time_count.Stop();

    auto faster_rcnn_out = xstream_output;
    static int frame_cnt = 0;
    if (need_render) {
      model_name2dump_fn[model_name](&bgr_img, faster_rcnn_out, input_image);
    }
    ret = fb_vio.FreeImgInfo(&data);
    HOBOT_CHECK(ret == 0) << "fb vio free image failed!!!";
    LOGD << "predict success: " << ++frame_cnt;
  }
#endif
#ifdef X3
  HbVioFbWrapperGlobal fb_vio("./configs/vio_onsemi0230_fb.json");
  auto ret = fb_vio.Init();
  HOBOT_CHECK(ret == 0) << "fb vio init failed!!!";

  Stopwatch time_count;
  time_count.Reset();
  while (getline(ifs, input_image)) {
    cv::Mat bgr_img = cv::imread(input_image);
    int width = bgr_img.cols;
    int height = bgr_img.rows;
    cv::Mat img_nv12;
    bgr_to_nv12(bgr_img.data, height, width, img_nv12);
    auto py_image_frame_ptr = fb_vio.GetImgInfo(img_nv12.data, width, height);
    HOBOT_CHECK(py_image_frame_ptr != nullptr) << "fb vio get image failed!!!";
    // vio_debug::print_info(data);
    std::string::size_type pos2;
    pos2 = input_image.rfind("/");
    if (pos2 != std::string::npos) input_image = input_image.substr(pos2 + 1);

    auto xstream_pyramid =
        std::make_shared<XStreamData<std::shared_ptr<ImageFrame>>>();
    xstream_pyramid->value = py_image_frame_ptr;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(xstream_pyramid);

    time_count.Start();
    auto xstream_output = faster_rcnn_method.DoProcess(input, param);
    time_count.Stop();

    auto faster_rcnn_out = xstream_output;
    static int frame_cnt = 0;
    if (need_render) {
      model_name2dump_fn[model_name](&bgr_img, faster_rcnn_out, input_image);
    }
    ret = fb_vio.FreeImgInfo(py_image_frame_ptr);
    HOBOT_CHECK(ret == 0) << "fb vio free image failed!!!";
    LOGD << "predict success: " << ++frame_cnt;
  }
#endif
  std::cout << "fasterrcnn time count: " << time_count.Str() << "\n";
  faster_rcnn_method.Finalize();
  // std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
