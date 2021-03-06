/**
 * @file method_factory.cpp
 * @author your name (you@domain.com)
 * @brief DO NOT MODIFY THIS FILE, WHICH IS AUTO GENERATED BY COMPILER
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "hobotxstream/method_factory.h"
#include "BehaviorMethod/BehaviorMethod.h"
#include "CNNMethod/CNNMethod.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "MOTMethod/MOTMethod.h"
#include "MergeMethod/MergeMethod.h"
#include "vote_method/vote_method.h"
#include <string>

#include "FilterSkipFrameMethod/FilterSkipFrameMethod.h"
#include "plate_vote_method/plate_vote_method.h"
#include "VehiclePlateMatchMethod/VehiclePlateMatchMethod.h"
#include "MultitaskPredictMethod/MultitaskPredictMethod.h"
#include "MultitaskPostProcessMethod/MultitaskPostProcessMethod.h"
#include "FaceSnapFilterMethod/FaceSnapFilterMethod.h"
#include "GradingMethod/GradingMethod.h"
#include "SnapShotMethod/SnapShotMethod.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("MOTMethod" == method_name) {
    return MethodPtr(new MOTMethod());
  } else if ("FasterRCNNMethod" == method_name) {
    return MethodPtr(new FasterRCNNMethod());
  } else if ("MergeMethod" == method_name) {
    return MethodPtr(new MergeMethod());
  } else if ("CNNMethod" == method_name) {
    return MethodPtr(new CNNMethod());
  } else if ("VoteMethod" == method_name) {
    return MethodPtr(new VoteMethod());
  } else if ("BehaviorMethod" == method_name) {
    return MethodPtr(new BehaviorMethod());
  } else if ("FilterSkipFrameMethod" == method_name) {
    return MethodPtr(new FilterSkipFrameMethod());
  } else if ("VehiclePlateMatchMethod" == method_name) {
    return MethodPtr(new VehiclePlateMatchMethod());
  } else if ("PlateVoteMethod" == method_name) {
    return MethodPtr(new PlateVoteMethod());
  } else if ("MultitaskPredictMethod" == method_name) {
    return MethodPtr(new MultitaskPredictMethod());
  } else if ("MultitaskPostProcessMethod" == method_name) {
    return MethodPtr(new MultitaskPostProcessMethod());
  } else if ("FaceSnapFilterMethod" == method_name) {
    return MethodPtr(new FaceSnapFilterMethod());
  } else if ("GradingMethod" == method_name) {
    return MethodPtr(new GradingMethod());
  } else if ("SnapShotMethod" == method_name) {
    return MethodPtr(new SnapShotMethod());
  } else {
    return MethodPtr();
  }
}
} //  namespace method_factory
} //  namespace xstream
