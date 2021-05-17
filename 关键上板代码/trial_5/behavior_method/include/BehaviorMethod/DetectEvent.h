#ifndef BEHAVIORMETHOD_DETECTEVENT_H_
#define BEHAVIORMETHOD_DETECTEVENT_H_

#include <vector>
#include <string>
#include <vector>
#include <memory>
#include "json/json.h"
#include "BehaviorMethod/BehaviorEvent.h"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxstream/simple_method.h"
#include "bpu_predict/bpu_predict_extension.h"

namespace xstream {

class DetectEvent : public BehaviorEvent {
 public:
  DetectEvent() {}
  virtual ~DetectEvent() {}

  int Init(const Json::Value &config,const std::string &cfg_path) override;
  void RunSingleFrame(const std::vector<BaseDataPtr> &frame_input,
                      std::vector<BaseDataPtr> &frame_output) override;
  bool IsEvent(hobot::vision::Landmarks kps) override;
  
  void FreeTensor(std::vector<BPU_TENSOR_S> &tensors);

 private:

  ////////
  std::string model_path_;
  std::shared_ptr<MYBPUModelWrapper> dnn_model_; //最重要的模型在这。
  int src_image_width_ = -1;
  int src_image_height_ = -1;

  int input_h_idx_, input_w_idx_, input_c_idx_;
  int model_input_height_, model_input_width_;
  bool dnn_is_sync_ = false;       // 默认异步,可配置
  bool dnn_run_with_roi_ = false;  // 默认非roi输入,可配置
  bool dnn_model_group_ = false;   // 是否开启group模式
  int dnn_model_group_id_ = 0;     // group模式，该模型的group id
  BPU_RUN_CTRL_S dnn_ctrl_;        // 运行的控制信息,core_id等

  //匹配track id 和一个60*17*3的数组
  std::unordered_map<uint32_t, float[60][16][2]> kps_map_;  // key:tarck_id
};

}  // namespace xstream

#endif  // BEHAVIORMETHOD_STANDEVENT_H_
