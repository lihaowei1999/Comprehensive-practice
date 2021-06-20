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
#include <string>
#include "MOTMethod/MOTMethod.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "GradingMethod/GradingMethod.h"
#include "SnapShotMethod/SnapShotMethod.h"
#include "CNNMethod/CNNMethod.h"
#include "vote_method/vote_method.h"
#include "FilterMethod/FilterMethod.h"
#include "MultitaskPredictMethod/MultitaskPredictMethod.h"
#include "MultitaskPostProcessMethod/MultitaskPostProcessMethod.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("MOTMethod" == method_name) {
    return MethodPtr(new MOTMethod());
  } else if ("FasterRCNNMethod" == method_name) {
    return MethodPtr(new FasterRCNNMethod());
  } else if ("FilterMethod" == method_name) {
    return MethodPtr(new FilterMethod());
  } else if ("GradingMethod" == method_name) {
    return MethodPtr(new GradingMethod());
  } else if ("SnapShotMethod" == method_name) {
    return MethodPtr(new SnapShotMethod());
  } else if ("CNNMethod" == method_name) {
    return MethodPtr(new CNNMethod());
  } else if ("VoteMethod" == method_name) {
    return MethodPtr(new VoteMethod());
  } else if ("MultitaskPredictMethod" == method_name) {
    return MethodPtr(new MultitaskPredictMethod());
  } else if ("MultitaskPostProcessMethod" == method_name) {
    return MethodPtr(new MultitaskPostProcessMethod());
  } else {
    return MethodPtr();
  }
}
}  //  namespace method_factory
}  //  namespace xstream