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

#ifndef BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_SEGMENT_POSTPROCESS_H_
#define BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_SEGMENT_POSTPROCESS_H_
#include <memory>
#include <string>
#include <vector>
#include "postprocess/detect_base_postprocess.h"
namespace hobot {
namespace bev {
  
/**
 * segment type
 */
enum class seg_Type { S8, S32, S64 };

class SegmentPostProcess : public DetectBasePostProcess {
 public:
  explicit SegmentPostProcess(const std::string &config_file);
  void Parse(std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<DnnParserResult>& result) override;

 private:
  int InitPostProcessInfo(const std::string &config_file);
  void ArgMaxChannel(const int &channel, const std::vector<float> &vec_data_seg,
                     std::vector<int> &vec_filter_seg);
  void ConvertOutputNCHW(void *src_ptr, void *dest_ptr,
                         const std::shared_ptr<DNNTensor> &output_tensor);
  seg_Type seg_type_ = seg_Type::S8;
  int segment_index_{-1};
  int src_image_height_{0};
  int src_image_width_{0};
  int basic_pyramid_image_width_{0};
  int basic_pyramid_image_height_{0};
};
}  // namespace bev
}  // namespace hobot
#endif  // BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_SEGMENT_POSTPROCESS_H_
