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

#ifndef BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_3DBOX_POSTPROCESS_H_
#define BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_3DBOX_POSTPROCESS_H_
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "postprocess/detect_base_postprocess.h"

namespace hobot {
namespace bev {

class Detect3dBoxPostProcess : public DetectBasePostProcess {
 public:
  explicit Detect3dBoxPostProcess(const std::string &config_file);
  void Parse(std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<HobotBevData>& result) override;

 private:
  int InitPostProcessInfo(const std::string &config_file);

  void DecodeTask(const int &task_i,
                  const std::shared_ptr<DNNTensor> &output_tensor_reg,
                  const std::shared_ptr<DNNTensor> &output_tensor_height,
                  const std::shared_ptr<DNNTensor> &output_tensor_dim,
                  const std::shared_ptr<DNNTensor> &output_tensor_rot,
                  const std::shared_ptr<DNNTensor> &output_tensor_vel,
                  const std::shared_ptr<DNNTensor> &output_tensor_heatmap,
                  std::vector<Bbox3D> &vec_bbox3d);
  void FilterHeatmap(std::vector<float> &vec_data_heatmap, int max,
                     const int &channels, std::vector<float> &vec_sort_data_hm,
                     std::vector<int> &vec_index_heatmap);
  void ConvertOutputNCHW(void *src_ptr, void *dest_ptr,
                         const std::shared_ptr<DNNTensor> &output_tensor);
  void FilterBboxParam(const int &channels,
                       const std::vector<float> &vec_topk_x,
                       const std::vector<float> &vec_topk_y,
                       std::vector<float> &vec_data_param,
                       std::vector<float> &vec_filter_param);
  void Decode3DBboxFilter(const int &channel_reg, const int &channel_dim,
                          const int &channel_rot,
                          const std::vector<float> &vec_topk_x,
                          const std::vector<float> vec_topk_y,
                          const std::vector<float> &vec_filter_reg,
                          const std::vector<float> &vec_filter_height,
                          const std::vector<float> &vec_filter_dim,
                          const std::vector<float> &vec_filter_rot,
                          const std::vector<float> &vec_sort_data_hm,
                          std::vector<Bbox3D> &vec_bbox3d);
  void Get3DBboxCorners(Bbox3D &bbox3d);
  void ConvertCornerToStandupBox(std::vector<Bbox3D> &bbox3d,
                                 std::vector<Bbox2D> &bbox2d_bev);
  void ResultNms(const int &i, std::vector<Bbox3D> &bbox3d);
  int Bev3dNms(const int &i, std::vector<Bbox3D> &bbox3d,
               std::vector<Bbox2D> &bbox2d_bev, bool suppress);
  void DecodetoEgo(std::vector<Bbox3D> &bbox3d);
  void CircleNms(const float &radius_threshhold, std::vector<Bbox3D> &bbox3d);
  void TestConvertOutputNCHW(const int &channel, void *src_ptr, void *dest_ptr);

 protected:
  int pre_max_size_{0};
  int post_max_size_{0};
  float box_score_threshold_{0.0f};
  std::vector<float> nms_threshold_;
  std::vector<float> bev_size_;
  std::vector<float> min_radius_;
  std::vector<std::string> nms_types_;
  std::vector<std::string> tasks_class_car_;
  std::vector<std::string> tasks_class_truck_;
  std::vector<std::string> tasks_class_bus_;
  std::vector<std::string> tasks_class_barrier_;
  std::vector<std::string> tasks_class_bicycle_;
  std::vector<std::string> tasks_class_pedestrian_;
  std::vector<std::string> class_names_;
  std::map<int, std::vector<std::string>> map_tasks_;

 private:
  float GetFloatByInt(int32_t value, uint32_t shift) {
    float ret_x = value;
    if (value != 0) {
      int *ix = reinterpret_cast<int *>(&ret_x);
      (*ix) -= shift * 0x00800000;
    }
    return ret_x;
  }
  bool nms_bbox_{false};
  int top_k_max_size_{0};
};
}  // namespace bev
}  // namespace hobot
#endif  // BEV_INCLUDE_BEV_BEV_MODEL_POSTPROCESS_DETECT_3DBOX_POSTPROCESS_H_
