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

#include "postprocess/postprocess.h"
#include "postprocess/detect_3dbox_postprocess.h"
#include "postprocess/segment_postprocess.h"
#include "jsonutil.h"
namespace hobot {
namespace bev {

BevPostProcess::BevPostProcess(const std::string &config_file) {
  InitPostProcessInfo(config_file);
}

int BevPostProcess::InitPostProcessInfo(const std::string &config_file) {
  rapidjson::Document json;
  if (JSONUtil::ParseJson(config_file, json) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Parse fail! config_file: " << config_file);
    return -1;
  }
  
  if (json.HasMember("postprocess") && json["postprocess"].HasMember("outputs")) {
    for (auto& v : json["postprocess"]["outputs"].GetArray()) {
      outputs_.push_back(v.GetString());
    }
  }

  if (!outputs_.empty()) {
    for (auto &output : outputs_) {
      if (output == "3dbox") {
        std::shared_ptr<Detect3dBoxPostProcess> sp_bbox_process =
            std::make_shared<Detect3dBoxPostProcess>(config_file);
        vec_sp_postprocess_.push_back(sp_bbox_process);
      }
      if (output == "segment") {
        std::shared_ptr<SegmentPostProcess> sp_segment_process =
            std::make_shared<SegmentPostProcess>(config_file);
        vec_sp_postprocess_.push_back(sp_segment_process);
      }
    }
  }
  return 0;
}

void BevPostProcess::OutputsPostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<HobotBevData>& result) {
  if (tensors.empty()) {
    return;
  }
  if (vec_sp_postprocess_.empty()) {
    return;
  }
  for (auto &sp : vec_sp_postprocess_) {
    sp->Parse(tensors, result);
  }
  return;
}

}  // namespace bev
}  // namespace hobot
