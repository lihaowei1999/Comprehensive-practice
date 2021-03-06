/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xsoul framework interface
 * @author    jianbo.qin
 * @email     jianbo.qin@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.14
 */
#include <cstring>

#include "hobotxsdk/xstream_capi.h"
#include "hobotxsdk/xstream_capi_type.h"
#include "hobotxsdk/xstream_error.h"
#include "hobotxsdk/xstream_sdk.h"

int HobotXStreamCapiSetConfig(HobotXStreamCapiHandle handler,
                              const char *cfg_name, const char *value) {
  if (NULL == handler || NULL == value) {
    return HOBOTXSTREAM_ERROR_INPUT_INVALID;
  }
  auto flow_ptr = static_cast<xstream::XStreamSDK *>(handler);
  return flow_ptr->SetConfig(cfg_name, value);
}

int HobotXStreamCapiUpdateConfig(HobotXStreamCapiHandle handler,
                                 const char *unique_name,
                                 const char *config_str) {
  if (NULL == handler || NULL == unique_name || NULL == config_str) {
    return HOBOTXSTREAM_ERROR_INPUT_INVALID;
  }
  auto flow_ptr = static_cast<xstream::XStreamSDK *>(handler);
  xstream::CommParamPtr parm_ptr =
      std::make_shared<xstream::SdkCommParam>(unique_name, config_str);
  return flow_ptr->UpdateConfig(unique_name, parm_ptr);
}

int HobotXStreamCapiGetConfig(HobotXStreamCapiHandle handler,
                              const char *unique_name, char *config_str,
                              uint32_t *length) {
  if (NULL == handler || NULL == unique_name) {
    return HOBOTXSTREAM_ERROR_INPUT_INVALID;
  }
  auto flow_ptr = static_cast<xstream::XStreamSDK *>(handler);
  std::string value;
  auto config = flow_ptr->GetConfig(unique_name);
  if (config) {
    value = config->Format();
  }
  if (config_str == NULL) {
    *length = (uint32_t)value.length() + 1;
    return 0;
  } else if (*length > value.size()) {
    memcpy(config_str, value.c_str(), value.size());
    config_str[value.size()] = '\0';
    return 0;
  } else {
    return HOBOTXSTREAM_ERROR_INPUT_INVALID;
  }
}
