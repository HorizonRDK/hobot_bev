// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "preprocess.h"
#include "jsonutil.h"

#include "dnn_node/util/image_proc.h"

namespace hobot {
namespace bev {

PreProcess::PreProcess(const std::string &config_file) {
  InitPredictInfo(config_file);
}

int PreProcess::InitPredictInfo(const std::string &config_file) {
  rapidjson::Document json;
  if (JSONUtil::ParseJson(config_file, json) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Parse fail! config_file: " << config_file);
    return -1;
  }

  if (json.HasMember("predict")) {
    if (json["predict"].HasMember("is_padding")) {
      is_padding_ = json["predict"]["is_padding"].GetBool();
    }
  }

  return 0;
}

int PreProcess::CvtData2Tensors(
    std::vector<std::shared_ptr<DNNTensor>> &input_tensors,
    Model *pmodel,
    const std::shared_ptr<FeedbackData>& sp_feedback_data
    ) {
  if (!pmodel || !sp_feedback_data ||
      sp_feedback_data->image_files.size() != 6 ||
      sp_feedback_data->image_files.size() != sp_feedback_data->points_files.size() ||
      sp_feedback_data->image_files.size() + sp_feedback_data->points_files.size() !=
      static_cast<size_t>(pmodel->GetInputCount())) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Invalid input data");
    return -1;
  }

  for (size_t idx = 0; idx < sp_feedback_data->image_files.size(); idx++) {
    const auto& image_file = sp_feedback_data->image_files.at(idx);
    if (access(image_file.c_str(), F_OK) != 0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "File is not exist! image_file: " << image_file);
      return -1;
    }
    
    hbDNNTensorProperties properties;
    pmodel->GetInputTensorProperties(properties, idx);
    int h_index = 1;
    int w_index = 2;
    if (properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
      h_index = 1;
      w_index = 2;
    } else if (properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
      h_index = 2;
      w_index = 3;
    }
    int in_h = properties.validShape.dimensionSize[h_index];
    int in_w = properties.validShape.dimensionSize[w_index];
    RCLCPP_INFO(rclcpp::get_logger("hobot_bev"),
                "The model input %d width is %d and height is %d",
                idx,
                in_w,
                in_h);

    // auto input = GetNV12Pyramid(image_file, in_h, in_w);
    auto input = hobot::dnn_node::ImageProc::GetNV12TensorFromNV12(image_file, in_h, in_w);
    input->properties = properties;
    input_tensors.emplace_back(input);
  }

  std::call_once(center_flag, [&, this, sp_feedback_data]() { PointsForInputTensors(pmodel, sp_feedback_data); });
  for (auto &tensor : input_tensors_) {
    input_tensors.emplace_back(tensor);
  }

  return 0;
}

int PreProcess::PointsForInputTensors(Model *pmodel, const std::shared_ptr<FeedbackData>& sp_feedback_data) {
  if (!sp_feedback_data || sp_feedback_data->points_files.size() != 6) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Invalid input data");
  }

  for (size_t idx = 0; idx < sp_feedback_data->points_files.size(); idx++) {
    const std::string& points_file = sp_feedback_data->points_files.at(idx);
    if (access(points_file.c_str(), F_OK) != 0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "File is not exist! points_file: " << points_file);
      return -1;
    }
    
    std::ifstream inxyfp(points_file, std::ios::in | std::ios::binary);
    if (!inxyfp) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Failed to open the file: " << points_file);
      return -1;
    }
    auto input_tensor = std::make_shared<DNNTensor>();
    pmodel->GetInputTensorProperties(input_tensor->properties, idx + 6);
    
    int h_index = 1;
    int w_index = 2;
    if (input_tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
      h_index = 1;
      w_index = 2;
    } else if (input_tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
      h_index = 2;
      w_index = 3;
    }
    int in_h = input_tensor->properties.validShape.dimensionSize[h_index];
    int in_w = input_tensor->properties.validShape.dimensionSize[w_index];
    RCLCPP_INFO(rclcpp::get_logger("hobot_bev"),
                "The model input %d width is %d and height is %d",
                idx + 6,
                in_w,
                in_h);

    auto ret = hbSysAllocCachedMem(&input_tensor->sysMem[0], in_h * in_w * 2 * 2);
    if (ret != 0) {
      return -1;
    }
    inxyfp.read(reinterpret_cast<char *>(input_tensor->sysMem[0].virAddr),
                in_h * in_w * 2 * 2);
    hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    input_tensors_.emplace_back(input_tensor);
    inxyfp.close();
  }
  return 0;
}

void PreProcess::FreeTensors(
    const std::vector<std::shared_ptr<DNNTensor>> &input_tensors) {
  if (is_padding_) {
    for (size_t i = 0; i < input_tensors.size(); i++) {
      auto input_tensor = input_tensors[i];
      hbSysFreeMem(&input_tensor->sysMem[0]);
      hbSysFreeMem(&input_tensor->sysMem[1]);
    }
  }
#ifdef TEST_UNDISTORT
  for (auto i = 0; i < 6; i++) {
    auto input_tensor = input_tensors[i];
    hbSysFreeMem(&input_tensor->sysMem[0]);
    hbSysFreeMem(&input_tensor->sysMem[1]);
  }
#endif
  return;
}

}  // namespace bev
}  // namespace hobot
