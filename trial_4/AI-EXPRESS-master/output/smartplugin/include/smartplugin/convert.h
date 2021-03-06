/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 03:42:16
 * @Version: v0.0.1
 * @Brief:  convert to xstream inputdata from input VioMessage
 * @Note:  extracted from xperson repo.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 02:57:24
 */
#ifndef INCLUDE_SMARTPLUGIN_CONVERT_H_
#define INCLUDE_SMARTPLUGIN_CONVERT_H_

#include <string>
#include "hobotxsdk/xstream_data.h"
#include "xproto_msgtype/vioplugin_data.h"
#include "smartplugin/traffic_info.h"

namespace horizon {
namespace iot {
using horizon::vision::xproto::basic_msgtype::VioMessage;

class Convertor {
 public:
  /**
   * @brief convert input VioMessage to xstream inputdata.
   * @param input input VioMessage
   * @param input image input name in xstream workflow config
   * @return xstream::InputDataPtr xstream input
   */
  static xstream::InputDataPtr ConvertInput(const VioMessage *input,
                                            std::string input_name = "image");

 public:
  static int image_compress_quality;
};

}  // namespace iot
}  // namespace horizon

#endif  //  INCLUDE_SMARTPLUGIN_CONVERT_H_
