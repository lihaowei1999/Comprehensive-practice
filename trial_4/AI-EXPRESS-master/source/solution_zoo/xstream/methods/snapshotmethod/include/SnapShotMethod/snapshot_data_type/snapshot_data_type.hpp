/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     snapshot_data_type header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.04.22
 */

#ifndef SNAPSHOTMETHOD_SNAPSHOT_DATA_TYPE_SNAPSHOT_DATA_TYPE_HPP_
#define SNAPSHOTMETHOD_SNAPSHOT_DATA_TYPE_SNAPSHOT_DATA_TYPE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "SnapShotMethod/image_utils/image_utils.hpp"
#include "hobotxstream/method.h"
#include "horizon/vision_type/vision_type.hpp"
#include "json/json.h"

namespace xstream {

using hobot::vision::Point;
using hobot::vision::Points;
using hobot::vision::SnapshotInfo;
using hobot::vision::SnapshotState;

typedef xstream::XStreamData<float> XStreamFloat;
typedef xstream::XStreamData<uint32_t> XStreamUint32;

typedef xstream::XStreamData<BBox> XStreamBBox;
typedef std::shared_ptr<XStreamBBox> XStreamBBoxPtr;

typedef std::shared_ptr<SnapshotState> SnapshotStatePtr;
typedef xstream::XStreamData<SnapshotStatePtr> XStreamSnapshotState;
typedef std::shared_ptr<XStreamSnapshotState> XStreamSnapshotStatePtr;

typedef SnapshotInfo<BaseDataPtr> SnapshotInfoBaseData;
typedef std::shared_ptr<SnapshotInfoBaseData> SnapshotInfoXStreamBaseDataPtr;
typedef xstream::XStreamData<SnapshotInfoXStreamBaseDataPtr>
    XStreamSnapshotInfo;

typedef std::shared_ptr<BaseDataVector> BaseDataVectorPtr;

struct SelectSnapShotInfo : SnapshotInfoBaseData {
  Point snap_base_point;
  float wide_scale = 0;
  float height_scale = 0;
  Points PointsToSnap(const Points &in) override;
};

#define SET_SNAPSHOT_METHOD_PARAM(json_cfg, type, key)          \
  do {                                                          \
    if (json_cfg.isMember(#key) && json_cfg[#key].is##type()) { \
      key = json_cfg[#key].as##type();                          \
      config_jv[#key] = key;                                    \
    }                                                           \
  } while (0)

struct SnapShotParam : public InputParam {
 public:
  explicit SnapShotParam(const std::string &content)
      : InputParam("SnapShotMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;
    unique_name_ = "SnapShotMethod";
  }

  virtual int UpdateParameter(const std::string &content);

  float scale_rate = 0;
  bool need_resize = true;
  unsigned output_width = 0;
  unsigned output_height = 0;
  bool snapshot_state_enable = false;
  bool save_original_image_frame = true;
  std::string snapshot_type = "select";
  Json::Value config_jv;
  std::string Format() override;
};

typedef std::shared_ptr<SelectSnapShotInfo> SelectSnapShotInfoPtr;

class SnapShotInfo {
 public:
  static SelectSnapShotInfoPtr GetSnapShotInfo(
      const ImageFramePtr &frame, const float &select_score,
      const XStreamBBoxPtr &pbbox, SnapShotParam *param,
      std::vector<BaseDataPtr> userdatas);

  static BaseDataVectorPtr GenerateSnapshotInfo(
      const std::vector<SelectSnapShotInfoPtr> &snap_infos,
      const int32_t &type);

  static BaseDataPtr GenerateWithoutSnapshot(const int32_t id);
};

}  // namespace xstream

#endif  // SNAPSHOTMETHOD_SNAPSHOT_DATA_TYPE_SNAPSHOT_DATA_TYPE_HPP_
