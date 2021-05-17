#include "BehaviorMethod/DetectEvent.h"
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxstream/profiler.h"
#include "hobotxstream/image_tools.h"
#include "bpu_predict/bpu_predict_extension.h"

namespace xstream {

int DetectEvent::Init(const Json::Value &config,const std::string &cfg_path) {
    BehaviorEvent::Init(config,cfg_path);//共用参数是滑动窗口大小和threshhold 这里不影响
    // 加载模型到dnn_model_
    auto get_parent_path = [](const std::string path) -> std::string {
        auto pos = path.rfind('/');
        if (std::string::npos != pos) {
            auto parent = path.substr(0, pos);
            return parent + "/";
        } else {
            return std::string("./");
        }
    };

  std::string model_path =
  config["model_file_path"].isString() ?
  config["model_file_path"].asString() : "";
  HOBOT_CHECK(model_path.size() > 0) << "must set model_file_path";
  std::string parent_path = get_parent_path(cfg_path);
  model_path_ = parent_path + model_path;
  //config["model_file_path"] = model_path_;

  dnn_model_ = std::make_shared<MYBPUModelWrapper>();//Wrapper定义在method.h里
  int ret = HB_BPU_loadModelFromFile(model_path_.c_str(), &dnn_model_->bpu_model);
  HOBOT_CHECK(ret == 0) << "load model failed: " << HB_BPU_getErrorName(ret);
  //加载模型参数
  int core_id = config["core_id"].isInt() ?
                config["core_id"].asInt() : 0;  // 默认为0
  HOBOT_CHECK(core_id <= 2 && core_id >= 0) << "core_id out of range";
  dnn_ctrl_.core_id = core_id;  // 注意：异步预测模式下指定core_id无效
  int resize_type = config["resize_type"].isInt() ?
                    config["resize_type"].asInt() : 0;  // 默认为0
  dnn_ctrl_.resize_type = resize_type;

  // 3. 获取模型输入大小
  HOBOT_CHECK(dnn_model_->bpu_model.input_num >= 1);
  ret = HB_BPU_getHW(dnn_model_->bpu_model.inputs[0].data_type,
                     &dnn_model_->bpu_model.inputs[0].shape,
                     &model_input_height_, &model_input_width_);
  HOBOT_CHECK(ret == 0) << "load model input size failed: "
      << HB_BPU_getErrorName(ret);
  LOGI << "model input hw= "<<model_input_height_<<' '<<model_input_width_;

  // 4. 获取模型输入hwc索引
  ret = HB_BPU_getHWCIndex(dnn_model_->bpu_model.inputs[0].data_type,
                           &dnn_model_->bpu_model.inputs[0].shape.layout,
                           &input_h_idx_, &input_w_idx_, &input_c_idx_);
  HOBOT_CHECK(ret == 0) << "load model input index failed: "
      << HB_BPU_getErrorName(ret);
   LOGI << "model input h w c idx = "<<input_h_idx_<<' '<<input_w_idx_<<' '<<input_c_idx_;

  // 身体倾斜角度
  /*
  if (config.isMember("body_slope")) {
    slope_ = config["body_slope"].asFloat();
    HOBOT_CHECK(slope_ > 0);
  }
  */
    BPU_MODEL_NODE_S &node = dnn_model_->bpu_model.inputs[0];
    LOGI<<"model input node shape"<<node.shape.d[0]<<" "<<node.shape.d[1]<<" "<<node.shape.d[2]<<' '<<node.shape.d[3];
    LOGI<<"model input node ndim "<<node.shape.ndim<<" input num "<<dnn_model_->bpu_model.input_num;// 正常是4个 如果是五个就……
  return 0;
}

void DetectEvent::RunSingleFrame(const std::vector<BaseDataPtr> &frame_input,
                      std::vector<BaseDataPtr> &frame_output) {
  LOGI<<"start Detect Event running";
  HOBOT_CHECK(frame_input.size() == 3);  // BBox, disappeard track_id and kps
  auto out_infos = std::make_shared<BaseDataVector>();
  frame_output.resize(1);  // behavior
  frame_output[0] = out_infos;
  int ret;
  std::vector<BaseDataPtr> boxes = std::static_pointer_cast<
      BaseDataVector>(frame_input[0])->datas_;
  std::vector<BaseDataPtr> disappeared_track_ids = std::static_pointer_cast<
      BaseDataVector>(frame_input[1])->datas_;
  std::vector<BaseDataPtr> kpses = std::static_pointer_cast<
      BaseDataVector>(frame_input[2])->datas_;
  if (boxes.size() != kpses.size()) {
    LOGE << "boxes.size(): " << boxes.size()
         << ", kpses.size(): " << kpses.size();
  }

  //估计是每个box对应一个人和17个kps，就每个人判断一遍姿势
  for (size_t i = 0; i < boxes.size(); ++i) {
    const auto &box = std::static_pointer_cast<
                          XStreamData<hobot::vision::BBox>>(boxes[i])->value;

    if (box.id < 0) {
      auto attribute_result = std::make_shared<XStreamData<
                                hobot::vision::Attribute<int32_t>>>();
      attribute_result->value.value = 0;
      out_infos->datas_.push_back(attribute_result);
      continue;
    }

    auto kps = std::static_pointer_cast<XStreamData<
                 hobot::vision::Landmarks>>(kpses[i]);

    hobot::vision::Landmarks mykps=kps->value;//我看isevent里是用kps->value拿到数值的

    //在下面runmodel
    std::vector<BPU_TASK_HANDLE> task_handle;
    //先把关节点数据转换成16*2的形式，即kpsdata[16][2]
    //把kpsdata更新到kps_map_里,需要用track_id匹配，获得track_kpsdata[60][16][2]
    //如果是已有的，就放在最后一帧，否则用自身填充所有60帧
    //然后再把这个track_kpsdata复制五份，填充到数组y里
    
    //最后初始化INPUT TENSOR
    //然后用memcpy把y复制到INPUT TENSOR里
    //最后处理disappeared_track_ids

    // 1、 将关节点数据转换成16*2
    float kpsdata[16][2];
    // 用mykps.values逐个复制
    {     
      kpsdata[0][0]=(mykps.values[12].x+mykps.values[11].x)/2;
      kpsdata[0][1]=(mykps.values[12].y+mykps.values[11].y)/2;
      kpsdata[15][0]=(mykps.values[6].x+mykps.values[5].x)/2;
      kpsdata[15][1]=(mykps.values[6].y+mykps.values[5].y)/2;
      kpsdata[1][0]=(kpsdata[15][0]+kpsdata[0][0])/2;
      kpsdata[1][1]=(kpsdata[15][1]+kpsdata[0][1])/2;

      kpsdata[2][0]=mykps.values[0].x;
      kpsdata[2][1]=mykps.values[0].y;      
      kpsdata[3][0]=mykps.values[5].x;
      kpsdata[3][1]=mykps.values[5].y;      
      kpsdata[4][0]=mykps.values[7].x;
      kpsdata[4][1]=mykps.values[7].y;   
      kpsdata[5][0]=mykps.values[9].x;
      kpsdata[5][1]=mykps.values[9].y; 
      kpsdata[6][0]=mykps.values[6].x;
      kpsdata[6][1]=mykps.values[6].y; 
      kpsdata[7][0]=mykps.values[8].x;
      kpsdata[7][1]=mykps.values[8].y; 
      kpsdata[8][0]=mykps.values[10].x;
      kpsdata[8][1]=mykps.values[10].y;
      kpsdata[9][0]=mykps.values[11].x;
      kpsdata[9][1]=mykps.values[11].y;     
      kpsdata[10][0]=mykps.values[13].x;
      kpsdata[10][1]=mykps.values[13].y;  
      kpsdata[11][0]=mykps.values[15].x;
      kpsdata[11][1]=mykps.values[15].y; 
      kpsdata[12][0]=mykps.values[12].x;
      kpsdata[12][1]=mykps.values[12].y;     
      kpsdata[13][0]=mykps.values[14].x;
      kpsdata[13][1]=mykps.values[14].y;  
      kpsdata[14][0]=mykps.values[16].x;
      kpsdata[14][1]=mykps.values[16].y;   
    }
    
    // 2、更新kps_map_
    uint32_t track_id = static_cast<uint32_t>(box.id);
    auto iter = kps_map_.find(track_id);
    if (iter == kps_map_.end()) {//如果这是这个track_id第一次出现
      for(int i1=0;i1<60;i1++ ){
        memcpy(kps_map_[track_id][i1],kpsdata,16*2*4);
      }     
    } else {
      // 已有该track_id，先集体往前移动一格
      memcpy(kps_map_[track_id][0],kps_map_[track_id][1],59*16*2*4);
      memcpy(kps_map_[track_id][59],kpsdata,16*2*4);
    }
    LOGI << "kps_map_update";
    // 3、把这个kps_map_[track_id]复制五份，填充到数组y里
    float y[300][16][2];    
    for(int i1=0;i1<5;i1++)
    {
      memcpy(y[60*i], kps_map_[track_id][0], 60*16*2*4);
    }

    //4、初始化Tensor
    // Tensor输入方式，调用HB_BPU_runModel完成预测，需要创建输入与输出tensor
    std::vector<BPU_TENSOR_S> input_tensors;
    std::vector<BPU_TENSOR_S> output_tensors;
    input_tensors.resize(1);
    output_tensors.resize(1);
    //初始化input,把300*16*2的kps数据放到这里
    BPU_TENSOR_S &pre_resize_tensor= input_tensors[0];
    {
      pre_resize_tensor.data_type = BPU_TYPE_TENSOR_F32;//float
      //int h_idx=1, w_idx=2, c_idx=3;//NHWC
      pre_resize_tensor.data_shape.ndim = 4;
      pre_resize_tensor.data_shape.d[0] = 25;
      pre_resize_tensor.data_shape.d[1] = 1;
      pre_resize_tensor.data_shape.d[2] = 1;
      pre_resize_tensor.data_shape.d[3] = 3;
      //pre_resize_tensor.data_shape.d[4] = 1;
      pre_resize_tensor.data_shape.layout=BPU_LAYOUT_NHWC;
      pre_resize_tensor.aligned_shape = pre_resize_tensor.data_shape;
      int length = pre_resize_tensor.data_shape.d[0]*pre_resize_tensor.data_shape.d[1]*pre_resize_tensor.data_shape.d[2]*pre_resize_tensor.data_shape.d[3];
      // alloc bpu-mem
      ret = HB_SYS_bpuMemAlloc(
          "in_data0", length*4, true, &pre_resize_tensor.data);
      if (ret != 0) {
        LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
      }
      // Copy y data to data0
      float *input_y = reinterpret_cast<float *>(pre_resize_tensor.data.virAddr);
      memcpy(input_y, y, length*4);
      HB_SYS_flushMemCache(&pre_resize_tensor.data, HB_SYS_MEM_CACHE_CLEAN);
    }
    LOGI << "pre_resize_tensor.data init success" ;

    //初始化output 60*1
    BPU_TENSOR_S &pre_output_tensor= output_tensors[0];
    {
      BPU_MODEL_NODE_S &node = dnn_model_->bpu_model.outputs[0];
      LOGI<<"output shape ndim"<<node.shape.ndim;
      LOGI<<"output shape ndim 0123 "<<node.shape.d[0]<<" "<<node.shape.d[1]<<" "<<node.shape.d[2]<<" "<<node.shape.d[3];
      pre_output_tensor.data_type = node.data_type;//int？
      switch (node.data_type)//到时候先print一下类型吧 要命
      {
      case BPU_TYPE_TENSOR_U8:
      case BPU_TYPE_TENSOR_S8:
        LOGI<<"OUTPUT shape = tensor u8/s8";
        break;
      case BPU_TYPE_TENSOR_F32:
      case BPU_TYPE_TENSOR_S32:
      case BPU_TYPE_TENSOR_U32:
        LOGI<<"OUTPUT shape = tensor 32";//大概还是float
        break;
      default:
        LOGI<<"OUTPUT shape != TENSOR";
        break;
      }
      //output NCHW
      pre_output_tensor.data_shape.ndim = 4;
      pre_output_tensor.data_shape.d[0] = 60;
      pre_output_tensor.data_shape.d[1] = 1;
      pre_output_tensor.data_shape.d[2] = 1;
      pre_output_tensor.data_shape.d[3] = 1;
      pre_output_tensor.data_shape.layout=node.shape.layout;
      pre_output_tensor.aligned_shape = pre_output_tensor.data_shape;

      // alloc bpu-mem
      ret = HB_SYS_bpuMemAlloc(
          "out_data0", 60*4, true, &pre_output_tensor.data);
      if (ret != 0) {
        LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
      }
      // Copy y data to data0
      HB_SYS_flushMemCache(&pre_output_tensor.data, HB_SYS_MEM_CACHE_CLEAN);
    }
    LOGI << "pre_output_tensor.data init success" ;
    task_handle.resize(1);
    // 申请input_tensor或output_tensor失败
    if (input_tensors.size() == 0 || output_tensors.size() == 0) {
      continue;
    }

    // 调用bpu-predict接口完成预测
    RUN_PROCESS_TIME_PROFILER("Run_Model");
    ret = HB_BPU_runModel(&dnn_model_->bpu_model,
                          input_tensors.data(), input_tensors.size(),
                          output_tensors.data(), output_tensors.size(),
                          &dnn_ctrl_, dnn_is_sync_, &task_handle[0]); //dnn_is_sync_=false 异步执行
    //原本异步还需要后处理 这里先略过 如果不行就改成同步的再试试 再不行就抄dnn

    if (ret != 0) {
      LOGE << "HB_BPU_runModel failed: " << HB_BPU_getErrorName(ret);
      // 释放input_tensor,output_tensor,task_handle
      // DnnPostProcessMethod中可通过这些字段判断是否需要后处理解析
      FreeTensor(input_tensors);
      FreeTensor(output_tensors);
      HB_BPU_releaseTask(&task_handle[0]);
      task_handle[0] = nullptr;
    }

    //////////////////////////// 之后把output的结果放到out
    int output_value = 0;//从60*1里找到最大的一个
    float max_output_value=0;
    float output_vec[60];//用来接数据

    float *src_output = reinterpret_cast<float *>(output_tensors[0].data.virAddr);
    memcpy(output_vec, src_output, 60*4);
    for(int i=0;i<60;i++)
    {
      LOGI<<"Score"<<i<<"="<<output_vec[i];
      if(output_vec[i]> max_output_value)
      {
        max_output_value = output_vec[i];
        output_value = i;//更新输出
      }
    }

    LOGI<<"Final result:"<<output_value;


    //测试输出
    char ch[100];
    sprintf(ch,"echo %d > ../../webservice/html/modules/data.html \n",rand());
    system(ch);
    //std::fstream fs;
    //fs.open("../../webservice/html/modules/data.html", std::ios::out);
    /*
    if(!fs.is_open())
    {
      LOGI<<"File opan failed";
    }
    fs << rand() << std::endl;
    fs.close();
    */


    auto attribute_result = std::make_shared<XStreamData<
                                hobot::vision::Attribute<int32_t>>>();
    attribute_result->value.value = output_value;
    out_infos->datas_.push_back(attribute_result);

    // 最后处理disappear
    for (const auto &data : disappeared_track_ids) {
      auto disappeared_track_id =
          std::static_pointer_cast<XStreamData<uint32_t>>(data)->value;
      auto iter = kps_map_.find(disappeared_track_id);
      if (iter != kps_map_.end()) {
        kps_map_.erase(iter);
      }
    }
  }//对box的for循环终点

}

bool DetectEvent::IsEvent(hobot::vision::Landmarks kps) {
  /*
  if (kps.values[5].score < skeleton_score_thres_ ||
      kps.values[6].score < skeleton_score_thres_ ||  // 肩
      kps.values[11].score < skeleton_score_thres_ ||
      kps.values[12].score < skeleton_score_thres_ ||  // 胯
      kps.values[13].score < skeleton_score_thres_ ||
      kps.values[14].score < skeleton_score_thres_ ||  // 膝
      kps.values[15].score < skeleton_score_thres_ ||
      kps.values[16].score < skeleton_score_thres_) {  // 脚
    return false;
  }
  */
  return true;
}


void DetectEvent::FreeTensor(std::vector<BPU_TENSOR_S> &tensors) {
  for (size_t i = 0; i < tensors.size(); i++) {
    BPU_TENSOR_S &tensor = tensors[i];
    switch (tensor.data_type) {
      case BPU_TYPE_IMG_Y:
      case BPU_TYPE_IMG_YUV_NV12:
      case BPU_TYPE_IMG_YUV444:
      case BPU_TYPE_IMG_RGB:
      case BPU_TYPE_IMG_BGR:
      case BPU_TYPE_IMG_BGRP:
      case BPU_TYPE_IMG_RGBP:
      case BPU_TYPE_TENSOR_U8:
      case BPU_TYPE_TENSOR_S8:
      case BPU_TYPE_TENSOR_F32:
      case BPU_TYPE_TENSOR_S32:
      case BPU_TYPE_TENSOR_U32:
        HB_SYS_bpuMemFree(&tensor.data);
        break;
      case BPU_TYPE_IMG_NV12_SEPARATE:
        HB_SYS_bpuMemFree(&tensor.data);
        HB_SYS_bpuMemFree(&tensor.data_ext);
        break;
      default:
        HOBOT_CHECK(0) << "not support data_type: " << tensor.data_type;
        break;
    }
  }
  tensors.clear();
}

}  // namespace xstream