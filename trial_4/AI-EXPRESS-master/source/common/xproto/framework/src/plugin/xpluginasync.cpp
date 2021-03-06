/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     plugin.cpp
 * \Author   Yingmin Li
 * \Mail     yingmin.li@horizon.ai
 * \Contributor Songshan Gong
 * \Mail     songshan.gong@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-07-28
 * \Brief    XPluginAsync implementation
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */
#include "xproto/plugin/xpluginasync.h"

#include "hobotlog/hobotlog.hpp"
#include "xproto/manager/msg_manager.h"
#include "xproto/plugin/xplugin.h"

namespace horizon {
namespace vision {
namespace xproto {

int XPluginAsync::Init() {
  LOGI << "XPluginAsync::Init";
  return 0;
}

void XPluginAsync::RegisterMsg(const std::string &type,
                               XProtoMessageFunc callback) {
  std::lock_guard<std::mutex> lock(msg_map_mutex_);
  HOBOT_CHECK(msg_map_.count(type) == 0)
      << "type:" << type << " already registered.";
  XPlugin::RegisterMsg(type);
  msg_map_[type] = callback;
}

void XPluginAsync::UnRegisterMsg(const std::string &type) {
  std::lock_guard<std::mutex> lock(msg_map_mutex_);
  auto iter = msg_map_.find(type);
  if (iter == msg_map_.end()) {
    return;
  }
  XPlugin::UnRegisterMsg(type);
  msg_map_.erase(iter);
}

int XPluginAsync::DeInit() {
  std::lock_guard<std::mutex> lock(msg_map_mutex_);
  for (auto &msginfo : msg_map_) {
    XPlugin::UnRegisterMsg(msginfo.first);
  }
  msg_map_.clear();
  return 0;
}

void XPluginAsync::OnMsg(XProtoMessagePtr msg) {
  // todo
  // 实现消息队列和流量控制
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (msg_handle_.GetTaskNum() >= 30) {
      LOGW << "Task Size: " << msg_handle_.GetTaskNum();
  }
  msg_handle_.PostTask(std::bind(&XPluginAsync::OnMsgDown, this, msg));
}

int XPluginAsync::GetPluginMsgCount() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return msg_handle_.GetTaskNum();
}

int XPluginAsync::GetPluginMsgLimit() {
  std::lock_guard<std::mutex> lock(msg_limit_mutex_);
  return msg_limit_count_;
}

void XPluginAsync::SetPluginMsgLimit(int msg_limit_count) {
  if (msg_limit_count <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(msg_limit_mutex_);
  msg_limit_count_ = msg_limit_count;
}

int XPluginAsync::GetMsgMonitorTime() {
  std::lock_guard<std::mutex> lock(msg_monitor_mutex_);
  return msg_monitor_time_;
}

void XPluginAsync::SetMsgMonitorTime(int msg_monitor_time) {
  if (msg_monitor_time < 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(msg_limit_mutex_);
  msg_monitor_time_ = msg_monitor_time;
}

void XPluginAsync::OnMsgDown(XProtoMessagePtr msg) {
  HOBOT_CHECK(msg_map_.count(msg->type()))
      << "No message type:" << msg->type() << " registered in " << desc();

  auto plugin_start = std::chrono::system_clock::now();
  msg_map_[msg->type()](msg);
  auto plugin_stop = std::chrono::system_clock::now();
  auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                      plugin_stop - plugin_start)
                      .count();
  if (interval >= msg_monitor_time_) {
    LOGW << "Plugin: " << desc() << ", MsgType = " << msg->type()
         << ", cost time = " << interval << "(ms)";
  }
}

XPluginAsync::XPluginAsync() {
  msg_handle_.CreatThread(1);
  LOGI << "XPluginAsync() cons";
}
XPluginAsync::XPluginAsync(int thread_num) {
  msg_handle_.CreatThread(thread_num);
  LOGD << "set XPluginAsync thread num = " << thread_num;
}

}  // namespace xproto
}  // namespace vision
}  // namespace horizon
