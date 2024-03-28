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

#ifndef BEV_INCLUDE_BEV_BEV_MODEL_PREPROCESS_H_
#define BEV_INCLUDE_BEV_BEV_MODEL_PREPROCESS_H_
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)

namespace hobot {
namespace bev {

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;
using hobot::dnn_node::NV12PyramidInput;

struct FeedbackData {
  // size is 6 and seq is:
  // image_front_left
  // image_front
  // image_front_right
  // image_back_left
  // image_back
  // image_back_right
  std::vector<std::string> image_files;
  std::vector<std::string> points_files;
};

class PreProcess {
 public:
  explicit PreProcess(const std::string &config_file);
  int CvtData2Tensors(
      std::vector<std::shared_ptr<DNNTensor>> &input_tensors,
      Model *pmodel,
      const std::shared_ptr<FeedbackData>& sp_feedback_data
      );

 private:
  int InitPredictInfo(const std::string &config_file);
  int PointsForInputTensors(Model *pmodel, const std::shared_ptr<FeedbackData>& sp_feedback_data);
  void FreeTensors(
      const std::vector<std::shared_ptr<DNNTensor>> &input_tensors);

  bool is_padding_{false};
  // test
  std::once_flag center_flag;
  std::vector<std::shared_ptr<DNNTensor>> input_tensors_;
};  // class PreProcess

}  // namespace bev
}  // namespace hobot

#endif  // BEV_INCLUDE_BEV_BEV_MODEL_PREPROCESS_H_
