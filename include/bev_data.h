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

#ifndef BEV_INCLUDE_BEV_BEV_DATA_H_
#define BEV_INCLUDE_BEV_BEV_DATA_H_

#include <memory>
#include <string>
#include <vector>

namespace hobot {
namespace bev {

struct Bbox2D {
  float x1, y1, x2, y2;
  float score;
  uint32_t cls;
};

struct Bbox3D {
  float score = 0.0;
  uint16_t cls = 0;
  std::string cls_str = "";
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float w = 0.0;
  float l = 0.0;
  float h = 0.0;
  float r = 0.0;
  std::vector<std::vector<float>> corners3d;  // undistort, camera ord
  Bbox3D() { corners3d.resize(8, std::vector<float>(3)); }
};

struct Parsing {
  std::vector<float> data;
  int32_t valid_h = 0;
  int32_t valid_w = 0;
};

struct HobotBevData {
  std::vector<std::vector<Bbox3D>> bbox3ds;
  Parsing seg;
};

}  // namespace bev
}  // namespace hobot

#endif  // BEV_INCLUDE_BEV_BEV_DATA_H_
