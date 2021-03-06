/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: PredictorFactory.h
 * @Brief: declaration of the PredictorFactory
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_PREDICTOR_PREDICTORFACTORY_H_
#define INCLUDE_CNNMETHOD_PREDICTOR_PREDICTORFACTORY_H_

#include "CNNMethod/Predictor/ImgInputPredictor.h"
#include "CNNMethod/Predictor/LmkInputPredictor.h"
#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/Predictor/RectInputPredictor.h"
#include "CNNMethod/Predictor/VehicleImgInputPredictor.h"
#include "CNNMethod/Predictor/LmkSeqInputPredictor.h"
#include "CNNMethod/Predictor/VidPredictor.h"

namespace xstream {

class PredictorFactory {
 public:
  static Predictor* GetPredictor(InputType input_type) {
    switch (input_type) {
      case InputType::RECT:
        return new RectInputPredictor();
      case InputType::LMK_IMG:
        return new LmkInputPredictor();
      case InputType::IMG:
        return new ImgInputPredictor();
      case InputType::PLATE_NUM_IMG:
        return new VehicleImgInputPredictor();
      case InputType::LMK_SEQ:
        return new LmkSeqInputPredictor();
      case InputType::VID:
        return new VidInputPredictor();
      case InputType::NONE: {
         HOBOT_CHECK(false) << "Invalid predictor type";
         return nullptr;
      }
    }
    return nullptr;
  }
};

}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_PREDICTOR_PREDICTORFACTORY_H_
