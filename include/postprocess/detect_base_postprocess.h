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

#ifndef BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_BASE_POSTPROCESS_H_
#define BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_BASE_POSTPROCESS_H_
#include <memory>
#include <string>
#include <vector>
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/output_parser/perception_common.h"

namespace hobot {
namespace bev {
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::output_parser::DnnParserResult;

class DetectBasePostProcess {
 public:
  DetectBasePostProcess() {}
  virtual ~DetectBasePostProcess() {}
  virtual void Parse(std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<DnnParserResult>& result) = 0;
};
}  // namespace bev
}  // namespace hobot
#endif  // BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_BASE_POSTPROCESS_H_
