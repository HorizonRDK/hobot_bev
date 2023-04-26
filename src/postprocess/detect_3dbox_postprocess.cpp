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

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "postprocess/detect_3dbox_postprocess.h"
#include "jsonutil.h"
namespace hobot {
namespace bev {

Detect3dBoxPostProcess::Detect3dBoxPostProcess(const std::string &config_file) {
  InitPostProcessInfo(config_file);
}
int Detect3dBoxPostProcess::InitPostProcessInfo(
    const std::string &config_file) {
  rapidjson::Document json;
  if (JSONUtil::ParseJson(config_file, json) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Parse fail! config_file: " << config_file);
    return -1;
  }
  
  if (json.HasMember("postprocess")) {
    if (json["postprocess"].HasMember("tasks")) {
      if (json["postprocess"]["tasks"].HasMember("car")) {
        for (auto& v : json["postprocess"]["tasks"]["car"].GetArray()) {
          tasks_class_car_.push_back(v.GetString());
        }
      }
    
      if (json["postprocess"]["tasks"].HasMember("truck")) {
        for (auto& v : json["postprocess"]["tasks"]["truck"].GetArray()) {
          tasks_class_truck_.push_back(v.GetString());
        }
      }

      if (json["postprocess"]["tasks"].HasMember("bus")) {
        for (auto& v : json["postprocess"]["tasks"]["bus"].GetArray()) {
          tasks_class_bus_.push_back(v.GetString());
        }
      }

      if (json["postprocess"]["tasks"].HasMember("barrier")) {
        for (auto& v : json["postprocess"]["tasks"]["barrier"].GetArray()) {
          tasks_class_barrier_.push_back(v.GetString());
        }
      }

      if (json["postprocess"]["tasks"].HasMember("bicycle")) {
        for (auto& v : json["postprocess"]["tasks"]["bicycle"].GetArray()) {
          tasks_class_bicycle_.push_back(v.GetString());
        }
      }

      if (json["postprocess"]["tasks"].HasMember("pedestrian")) {
        for (auto& v : json["postprocess"]["tasks"]["pedestrian"].GetArray()) {
          tasks_class_pedestrian_.push_back(v.GetString());
        }
      }

    }

    if (json["postprocess"].HasMember("box_score_threshold")) {
      box_score_threshold_ = json["postprocess"]["box_score_threshold"].GetFloat();
    }

    if (json["postprocess"].HasMember("top_k_max_size")) {
      top_k_max_size_ = json["postprocess"]["top_k_max_size"].GetInt();
    }

    if (json["postprocess"].HasMember("post_max_size")) {
      post_max_size_ = json["postprocess"]["post_max_size"].GetInt();
    }

    if (json["postprocess"].HasMember("pre_max_size")) {
      pre_max_size_ = json["postprocess"]["pre_max_size"].GetInt();
    }

    if (json["postprocess"].HasMember("class_names")) {
      for (auto& v : json["postprocess"]["class_names"].GetArray()) {
        class_names_.push_back(v.GetString());
      }
    }

    if (json["postprocess"].HasMember("nms_threshold")) {
      for (auto& v : json["postprocess"]["nms_threshold"].GetArray()) {
        nms_threshold_.push_back(v.GetFloat());
      }
    }

    if (json["postprocess"].HasMember("bev_size")) {
      for (auto& v : json["postprocess"]["bev_size"].GetArray()) {
        bev_size_.push_back(v.GetFloat());
      }
    }

    if (json["postprocess"].HasMember("nms_type")) {
      for (auto& v : json["postprocess"]["nms_type"].GetArray()) {
        nms_types_.push_back(v.GetString());
      }
    }

    if (json["postprocess"].HasMember("min_radius")) {
      for (auto& v : json["postprocess"]["min_radius"].GetArray()) {
        min_radius_.push_back(v.GetFloat());
      }
    }

    if (json["postprocess"].HasMember("nms_bbox")) {
      nms_bbox_ = json["postprocess"]["nms_bbox"].GetBool();
    }
  }

  map_tasks_.insert(make_pair(0, tasks_class_car_));
  map_tasks_.insert(make_pair(1, tasks_class_truck_));
  map_tasks_.insert(make_pair(2, tasks_class_bus_));
  map_tasks_.insert(make_pair(3, tasks_class_barrier_));
  map_tasks_.insert(make_pair(4, tasks_class_bicycle_));
  map_tasks_.insert(make_pair(5, tasks_class_pedestrian_));

  return 0;
}

void Detect3dBoxPostProcess::Parse(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<DnnParserResult>& result) {
  if (result) {
    // TODO add data to result
  }

  std::stringstream ss;
  for (size_t i = 0; i < map_tasks_.size(); i++) {
    std::vector<Bbox3D> bbox3d;
    // test all+1
    DecodeTask(
        i, tensors[map_tasks_.size() * i + 1],
        tensors[map_tasks_.size() * i + 2], tensors[map_tasks_.size() * i + 3],
        tensors[map_tasks_.size() * i + 4], tensors[map_tasks_.size() * i + 5],
        tensors[map_tasks_.size() * i + 6], bbox3d);

    ResultNms(i, bbox3d);
    for (const auto& bbox : bbox3d) {
      ss << "task: " << i << " bbox:"
        << " " << bbox.score
        << " " << bbox.cls
        << " " << bbox.x
        << " " << bbox.y
        << " " << bbox.z
        << " " << bbox.w
        << " " << bbox.l
        << " " << bbox.h
        << " " << bbox.r
        << "\n";
    }
  }
  printf("%s\n", ss.str().data());
  std::ofstream ofs("ofs.txt");
  ofs << ss.str();
}

void Detect3dBoxPostProcess::DecodeTask(
    const int &task_i, const std::shared_ptr<DNNTensor> &output_tensor_reg,
    const std::shared_ptr<DNNTensor> &output_tensor_height,
    const std::shared_ptr<DNNTensor> &output_tensor_dim,
    const std::shared_ptr<DNNTensor> &output_tensor_rot,
    const std::shared_ptr<DNNTensor> &output_tensor_vel,
    const std::shared_ptr<DNNTensor> &output_tensor_heatmap,
    std::vector<Bbox3D> &vec_bbox3d) {
  auto &aligned_shape_reg = output_tensor_reg->properties.alignedShape;
  auto &aligned_shape_height = output_tensor_height->properties.alignedShape;
  auto &aligned_shape_dim = output_tensor_dim->properties.alignedShape;
  auto &aligned_shape_rot = output_tensor_rot->properties.alignedShape;
  auto &aligned_shape_vel = output_tensor_vel->properties.alignedShape;
  auto &aligned_shape_hm = output_tensor_heatmap->properties.alignedShape;
  auto height = aligned_shape_hm.dimensionSize[2];
  auto width = aligned_shape_hm.dimensionSize[3];

  auto channel_reg = aligned_shape_reg.dimensionSize[1];

  auto channel_height = aligned_shape_height.dimensionSize[1];

  auto channel_dim = aligned_shape_dim.dimensionSize[1];

  auto channel_rot = aligned_shape_rot.dimensionSize[1];

  auto channel_vel = aligned_shape_vel.dimensionSize[1];

  auto channel_hm = aligned_shape_hm.dimensionSize[1];
  std::vector<float> vec_data_reg(height * width * channel_reg);
  std::vector<float> vec_data_height(height * width * channel_height);
  std::vector<float> vec_data_dim(height * width * channel_dim);
  std::vector<float> vec_data_rot(height * width * channel_rot);
  std::vector<float> vec_data_vel(height * width * channel_vel);
  std::vector<float> vec_data_heatmap(height * width * channel_hm);
  ConvertOutputNCHW(output_tensor_reg->sysMem[0].virAddr, vec_data_reg.data(),
                    output_tensor_reg);
  ConvertOutputNCHW(output_tensor_height->sysMem[0].virAddr,
                    vec_data_height.data(), output_tensor_height);
  ConvertOutputNCHW(output_tensor_dim->sysMem[0].virAddr, vec_data_dim.data(),
                    output_tensor_dim);
  ConvertOutputNCHW(output_tensor_rot->sysMem[0].virAddr, vec_data_rot.data(),
                    output_tensor_rot);
  ConvertOutputNCHW(output_tensor_vel->sysMem[0].virAddr, vec_data_vel.data(),
                    output_tensor_vel);
  ConvertOutputNCHW(output_tensor_heatmap->sysMem[0].virAddr,
                    vec_data_heatmap.data(), output_tensor_heatmap);
// heatmap sigmoid
  // cv::Mat box_hm_data_mat(height * width * channel_hm, 1, CV_32FC1,
  //                         vec_data_heatmap.data());
  cv::Mat box_hm_data_mat(height * width, channel_hm, CV_32FC1,
                          vec_data_heatmap.data());
  cv::Mat box_heatmap_data;

  cv::exp(box_hm_data_mat * (-1), box_heatmap_data);
  box_heatmap_data = 1 / (1 + box_heatmap_data);

  vec_data_heatmap.clear();
  for (int row = 0; row < box_heatmap_data.rows; row++) {
    float *ptr = box_heatmap_data.ptr<float>(row);
    for (int col = 0; col < box_heatmap_data.cols; col++) {
      // ofscore << ptr[col] << " ";
      vec_data_heatmap.emplace_back(ptr[col]);
    }
    // ofscore << std::endl;
  }
// ofscore.close();
  std::vector<float> vec_sort_data_hm;
  std::vector<int> vec_index_heatmap;

  FilterHeatmap(vec_data_heatmap, top_k_max_size_, channel_hm, vec_sort_data_hm,
                vec_index_heatmap);
  std::vector<float> vec_topk_y;
  std::vector<float> vec_topk_x;
  for (auto iter = vec_index_heatmap.begin(); iter != vec_index_heatmap.end();
       iter++) {
    // 只适用于channel_hm<=2，如果>2,需要将channel_hm带入计算
    vec_topk_y.emplace_back((*iter) / width > 128 ? ((*iter) / width - 128)
                                                  : (*iter) / width);
    vec_topk_x.emplace_back((*iter) % width);
    Bbox3D bbox;
    int tmp_cls = (*iter) / (width * height);
    for (auto i = 0; i < class_names_.size(); i++) {
      if (map_tasks_[task_i][tmp_cls] == class_names_[i]) {
        bbox.cls = i;
        break;
      }
    }
    vec_bbox3d.emplace_back(bbox);
  }

  std::vector<float> vec_filter_reg;
  std::vector<float> vec_filter_height;
  std::vector<float> vec_filter_dim;
  std::vector<float> vec_filter_rot;
  std::vector<float> vec_filter_vel;

  FilterBboxParam(channel_reg, vec_topk_x, vec_topk_y, vec_data_reg,
                  vec_filter_reg);
  FilterBboxParam(channel_height, vec_topk_x, vec_topk_y, vec_data_height,
                  vec_filter_height);
  FilterBboxParam(channel_dim, vec_topk_x, vec_topk_y, vec_data_dim,
                  vec_filter_dim);
  FilterBboxParam(channel_rot, vec_topk_x, vec_topk_y, vec_data_rot,
                  vec_filter_rot);
  FilterBboxParam(channel_vel, vec_topk_x, vec_topk_y, vec_data_vel,
                  vec_filter_vel);

  Decode3DBboxFilter(channel_reg, channel_dim, channel_rot, vec_topk_x,
                     vec_topk_y, vec_filter_reg, vec_filter_height,
                     vec_filter_dim, vec_filter_rot, vec_sort_data_hm,
                     vec_bbox3d);
}

void Detect3dBoxPostProcess::Decode3DBboxFilter(
    const int &channel_reg, const int &channel_dim, const int &channel_rot,
    const std::vector<float> &vec_topk_x, const std::vector<float> vec_topk_y,
    const std::vector<float> &vec_filter_reg,
    const std::vector<float> &vec_filter_height,
    const std::vector<float> &vec_filter_dim,
    const std::vector<float> &vec_filter_rot,
    const std::vector<float> &vec_sort_data_hm,
    std::vector<Bbox3D> &vec_bbox3d) {
  for (auto i = 0; i < vec_bbox3d.size(); i++) {
    vec_bbox3d[i].score = vec_sort_data_hm[i];
    vec_bbox3d[i].x = (vec_filter_reg[i * channel_reg] + vec_topk_x[i]);
    vec_bbox3d[i].y = (vec_filter_reg[i * channel_reg + 1] + vec_topk_y[i]);
    vec_bbox3d[i].z = vec_filter_height[i];
    if (nms_bbox_) {
      vec_bbox3d[i].w = exp(vec_filter_dim[i * channel_dim]);
      vec_bbox3d[i].l = exp(vec_filter_dim[i * channel_dim + 1]);
      vec_bbox3d[i].h = exp(vec_filter_dim[i * channel_dim + 2]);
    } else {
      vec_bbox3d[i].w = vec_filter_dim[i * channel_dim];
      vec_bbox3d[i].l = vec_filter_dim[i * channel_dim + 1];
      vec_bbox3d[i].h = vec_filter_dim[i * channel_dim + 2];
    }
    vec_bbox3d[i].r = atan2(vec_filter_rot[i * channel_rot],
                            vec_filter_rot[i * channel_rot + 1]);
    Get3DBboxCorners(vec_bbox3d[i]);
  }
  for (auto iter = vec_bbox3d.begin(); iter != vec_bbox3d.end();) {
    if ((*iter).score <= box_score_threshold_ || (*iter).x < 0 ||
        (*iter).x > 128 || (*iter).y < 0 || (*iter).y > 128) {
      iter = vec_bbox3d.erase(iter);
    } else {
      iter++;
    }
  }

  if (vec_bbox3d.size() > pre_max_size_) {
    vec_bbox3d.resize(pre_max_size_);
  }
}

void Detect3dBoxPostProcess::Get3DBboxCorners(Bbox3D &bbox3d) {
  float x = bbox3d.x;
  float y = bbox3d.y;
  float z = bbox3d.z;
  float h = bbox3d.h / 2;
  float w = bbox3d.w / 2;
  float l = bbox3d.l / 2;
  float yaw = bbox3d.r;
  std::vector<std::vector<float>> corners = {
      {-l, -l, l, l, -l, -l, l, l},   // x
      {w, -w, -w, w, w, -w, -w, w},   // y
      {-h, -h, -h, -h, h, h, h, h}};  // z

  float c = cos(yaw);
  float s = sin(yaw);
  std::vector<std::vector<float>> rot = {{c, -s, 0.0}, {s, c, 0.0}, {0, 0, 1}};

  std::vector<float> ct = {x, y, z};
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 8; ++row) {
      bbox3d.corners3d[row][col] = rot[col][0] * corners[0][row] +
                                   rot[col][1] * corners[1][row] +
                                   rot[col][2] * corners[2][row] + ct[col];
    }
  }
}

void Detect3dBoxPostProcess::ConvertCornerToStandupBox(
    std::vector<Bbox3D> &bbox3d, std::vector<Bbox2D> &bbox2d_bev) {
  auto bbox3d_size = bbox3d.size();
  for (size_t i = 0; i < bbox3d_size; i++) {
    // x_top_left = min(x1, x2, x3, x4)
    bbox2d_bev[i].x1 =
        std::min({bbox3d[i].corners3d[0][0], bbox3d[i].corners3d[1][0],
                  bbox3d[i].corners3d[2][0], bbox3d[i].corners3d[3][0]},
                 std::less<float>());

    // y_top_left = min(y1, y2, y3, y4)
    bbox2d_bev[i].y1 =
        std::min({bbox3d[i].corners3d[0][1], bbox3d[i].corners3d[1][1],
                  bbox3d[i].corners3d[2][1], bbox3d[i].corners3d[3][1]},
                 std::less<float>());

    // x_bottom_right = max(x1, x2, x3, x4)
    bbox2d_bev[i].x2 =
        std::max({bbox3d[i].corners3d[0][0], bbox3d[i].corners3d[1][0],
                  bbox3d[i].corners3d[2][0], bbox3d[i].corners3d[3][0]},
                 std::less<float>());

    // y_bottom_right = max(y1, y2, y3, y4)
    bbox2d_bev[i].y2 =
        std::max({bbox3d[i].corners3d[0][1], bbox3d[i].corners3d[1][1],
                  bbox3d[i].corners3d[2][1], bbox3d[i].corners3d[3][1]},
                 std::less<float>());

    bbox2d_bev[i].score = bbox3d[i].score;
    bbox2d_bev[i].cls = bbox3d[i].cls;
  }
}

void Detect3dBoxPostProcess::ResultNms(const int &i,
                                       std::vector<Bbox3D> &bbox3d) {
  if (bbox3d.size() == 0) {
    return;
  }
  if (nms_types_[i] == "circle") {
    CircleNms(min_radius_[i], bbox3d);
  } else {
    std::vector<Bbox2D> bbox2d_bev(bbox3d.size());
    ConvertCornerToStandupBox(bbox3d, bbox2d_bev);
    Bev3dNms(i, bbox3d, bbox2d_bev, true);
  }
}
int Detect3dBoxPostProcess::Bev3dNms(const int &i, std::vector<Bbox3D> &bbox3d,
                                     std::vector<Bbox2D> &bbox2d_bev,
                                     bool suppress) {
  auto bbox3d_size = bbox3d.size();
  if (bbox3d_size == 0) {
    return 0;
  }
  auto greater = [](const Bbox3D &a, const Bbox3D &b) {
    return a.score > b.score;
  };
  std::stable_sort(bbox3d.begin(), bbox3d.end(), greater);

  const float iou_thresh = nms_threshold_[i];

  std::vector<float> areas(bbox3d_size);
  std::vector<bool> skip(bbox3d_size, false);
  for (size_t i = 0; i < bbox3d_size; ++i) {
    areas[i] = (bbox2d_bev[i].x2 - bbox2d_bev[i].x1) *
               (bbox2d_bev[i].y2 - bbox2d_bev[i].y1);
  }

  size_t count = 0U;
  for (size_t i = 0U; count < bbox3d_size && i < skip.size(); ++i) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (bbox2d_bev[i].cls != bbox2d_bev[j].cls) {
          continue;
        }
      }
      float xx1 = std::max(bbox2d_bev[i].x1, bbox2d_bev[j].x1);
      float yy1 = std::max(bbox2d_bev[i].y1, bbox2d_bev[j].y1);
      float xx2 = std::min(bbox2d_bev[i].x2, bbox2d_bev[j].x2);
      float yy2 = std::min(bbox2d_bev[i].y2, bbox2d_bev[j].y2);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);
      if (area_intersection_valid) {
        float iou =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou > iou_thresh) {
          skip[j] = true;
        }
      }
    }
    bbox3d[count] = bbox3d[i];
    ++count;
  }
  count = (count < post_max_size_) ? count : post_max_size_;
  bbox3d.resize(count);
  return 0;
}
void Detect3dBoxPostProcess::CircleNms(const float &radius_threshhold,
                                       std::vector<Bbox3D> &bbox3d) {
  auto bbox3d_size = bbox3d.size();
  if (bbox3d_size == 0) {
    return;
  }
  auto greater = [](const Bbox3D &a, const Bbox3D &b) {
    return a.score > b.score;
  };
  std::stable_sort(bbox3d.begin(), bbox3d.end(), greater);

  std::vector<bool> skip(bbox3d_size, false);
  size_t count = 0U;
  for (size_t i = 0U; count < bbox3d_size && i < skip.size(); ++i) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      float x = bbox3d[i].x - bbox3d[j].x;
      float y = bbox3d[i].y - bbox3d[j].y;
      float dist = x * x + y * y;
      if (dist <= radius_threshhold) {
        skip[j] = true;
      }
    }
    bbox3d[count] = bbox3d[i];
    ++count;
  }
  count = (count < post_max_size_) ? count : post_max_size_;
  bbox3d.resize(count);
  return;
}
void Detect3dBoxPostProcess::FilterBboxParam(
    const int &channels, const std::vector<float> &vec_topk_x,
    const std::vector<float> &vec_topk_y, std::vector<float> &vec_data_param,
    std::vector<float> &vec_filter_param) {
  for (size_t i = 0; i < vec_topk_x.size(); i++) {
    for (auto c = 0; c < channels; c++) {
      vec_filter_param.emplace_back(vec_data_param[(
          128 * vec_topk_y[i] + vec_topk_x[i] + c * 128 * 128)]);
    }
  }
}
void Detect3dBoxPostProcess::FilterHeatmap(
    std::vector<float> &vec_data_heatmap, int max, const int &channels,
    std::vector<float> &vec_sort_data_hm, std::vector<int> &vec_index_heatmap) {
  int size = max > vec_data_heatmap.size() ? vec_data_heatmap.size() : max;
  float min_value =
      *min_element(vec_data_heatmap.begin(), vec_data_heatmap.end()) - 1;
  for (int i = 0; i < size; i++) {
    int max_p = max_element(vec_data_heatmap.begin(), vec_data_heatmap.end()) -
                vec_data_heatmap.begin();
    if (vec_data_heatmap[max_p] ==
        min_value)  // 如果取得的最大值==设定的最小值，证明取完了
      break;
    vec_sort_data_hm.emplace_back(vec_data_heatmap[max_p]);
    vec_index_heatmap.emplace_back(max_p);
    vec_data_heatmap[max_p] = min_value;
  }
}

void Detect3dBoxPostProcess::ConvertOutputNCHW(
    void *src_ptr, void *dest_ptr,
    const std::shared_ptr<DNNTensor> &output_tensor) {
  auto &real_shape = output_tensor->properties.validShape;
  auto scale = output_tensor->properties.scale.scaleData;
  auto elem_size = 4;
  if (output_tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_S32) {
    elem_size = 4;
  }
  float tmp_float_value;
  int32_t tmp_int32_value;
  for (int cc = 0; cc < real_shape.dimensionSize[1]; ++cc) {
    for (int hh = 0; hh < real_shape.dimensionSize[2]; hh++) {
      for (int ww = 0; ww < real_shape.dimensionSize[3]; ++ww) {
        tmp_int32_value =
            *(reinterpret_cast<int32_t *>(src_ptr) +
              real_shape.dimensionSize[2] * real_shape.dimensionSize[3] * cc +
              real_shape.dimensionSize[3] * hh + ww);
        tmp_float_value = scale[cc] * tmp_int32_value;
        *(reinterpret_cast<float *>(dest_ptr)) = tmp_float_value;
        dest_ptr = reinterpret_cast<char *>(dest_ptr) + elem_size;
      }
    }
  }
}

void Detect3dBoxPostProcess::TestConvertOutputNCHW(const int &channel,
                                                   void *src_ptr,
                                                   void *dest_ptr) {
  auto elem_size = 4;
  float tmp_float_value;
  for (int cc = 0; cc < channel; ++cc) {
    for (int hh = 0; hh < 128; hh++) {
      for (int ww = 0; ww < 128; ++ww) {
        tmp_float_value = *(reinterpret_cast<float *>(src_ptr) +
                            128 * 128 * cc + 128 * hh + ww);
        *(reinterpret_cast<float *>(dest_ptr)) = tmp_float_value;
        dest_ptr = reinterpret_cast<char *>(dest_ptr) + elem_size;
      }
    }
  }
}

}  // namespace bev
}  // namespace hobot
