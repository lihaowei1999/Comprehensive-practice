/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 *  Created by jianbo on 12/10/18.
 */

#ifndef XSTREAM_FRAMEWORK_JSON_KEY_H
#define XSTREAM_FRAMEWORK_JSON_KEY_H

namespace xstream {
const char* const kMaxRunCount = "max_running_count";
const char* const kWorkflow = "workflow";
const char* const kMethodCfgPath = "method_config_file";
const char* const kMethodCfgString = "method_config";
const char* const kMethodType = "method_type";
const char* const kUniqueName = "unique_name";
const char* const kTimeOutDuration = "timeout_duration";
const char* const kThreadNum = "thread_count";
const char* const kTheadList = "thread_list";
const char* const kInputs = "inputs";
const char* const kOutputs = "outputs";
const char* const kOutputName = "output_type";
const char* const kSingleOutputName = "__NODE_WHOLE_OUTPUT__";
const char* const kNodeOutputName = "__NODE_SINGLE_OUTPUT__";
const char* const kOptional = "optional";
const char* const kSchedUpper = "sched_upper";
const char* const kSchedDown = "sched_down";
const char* const kPriority = "priority";
const char* const kThreadPriority = "thread_priority";
const char* const kPolicy = "policy";
const char* const kSourceNum = "source_number";
}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_JSON_KEY_H
