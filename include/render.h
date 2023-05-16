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

#ifndef BEV_RENDER_H
#define BEV_RENDER_H

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"
#include "bev_data.h"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace hobot {
namespace bev {

struct RenderPara {
  std::vector<std::string> image_infos {
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT"
  };

  std::vector<std::string> mask_infos {
  	"background",
  	"vehicle_lane",
  	"humanoid_road",
  	"curb"
  };

  std::vector<std::array<unsigned char, 3>> mask_colors {
    std::array<unsigned char, 3>{0,0,0},
    std::array<unsigned char, 3>{255,255,255},
    std::array<unsigned char, 3>{0,0,255},
    std::array<unsigned char, 3>{255,0,0}
  };

  std::map<std::string, cv::Scalar> rect_colors {
    {"car", cv::Scalar(226,19,128)},
    {"truck", cv::Scalar(0,255,255)},
    {"trailer", cv::Scalar(84,198,42)},
    {"bus", cv::Scalar(0,255,200)},
    {"construction_vehicle", cv::Scalar(125,125,0)},
    {"bicycle", cv::Scalar(125,0,125)},
    {"motorcycle", cv::Scalar(0,0,255)},
    {"pedestrian", cv::Scalar(0,255,0)},
    {"traffic_cone", cv::Scalar(200,0,0)},
    {"barrier", cv::Scalar(0,19,128)}
  };

  int layout_w_num = 3;
  int layout_h_num = 3;

  int ipm_seg_size_w = 400;
  int ipm_seg_size_h = 200;
  int ipm_bev_size_w = 128;
  int ipm_bev_size_h = 128;
  int ipm_car_x = 64;
  int ipm_car_y = 64;
// ipm 128*128, car in center
  int car_width = 8;
  int car_heigth = 5;
  
  float ipm_x_meter_per_pixel = 0.8;
  float ipm_y_meter_per_pixel = 0.8;

  float img_Weighted_{0.1};
  float segment_Weighted_{0.9};
  int sawtooth_ksize_{3};
  bool sawtooth_pro_{true};
};

class BevRender {
 public:
  int Render(
    const std::vector<std::string>& image_files,
    const std::shared_ptr<HobotBevData>& det_result,
    cv::Mat& mat_bg);

 private:
  RenderPara render_para;
};

}
}
#endif  // BEV_RENDER_H
