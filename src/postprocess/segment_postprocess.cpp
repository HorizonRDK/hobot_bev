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

#include "postprocess/segment_postprocess.h"

#include "jsonutil.h"
namespace hobot {
namespace bev {
SegmentPostProcess::SegmentPostProcess(const std::string &config_file) {
  InitPostProcessInfo(config_file);
}

int SegmentPostProcess::InitPostProcessInfo(const std::string &config_file) {
  rapidjson::Document json;
  if (JSONUtil::ParseJson(config_file, json) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "Parse fail! config_file: " << config_file);
    return -1;
  }

  if (json.HasMember("postprocess")) {
    if (json["postprocess"].HasMember("basic_pyramid_image_height")) {
      basic_pyramid_image_height_ = json["postprocess"]["basic_pyramid_image_height"].GetInt();
    }
    if (json["postprocess"].HasMember("basic_pyramid_image_width")) {
      basic_pyramid_image_width_ = json["postprocess"]["basic_pyramid_image_width"].GetInt();
    }
    if (json["postprocess"].HasMember("src_image_height")) {
      src_image_height_ = json["postprocess"]["src_image_height"].GetInt();
    }
    if (json["postprocess"].HasMember("src_image_width")) {
      src_image_width_ = json["postprocess"]["src_image_width"].GetInt();
    }

    if (json["postprocess"].HasMember("outputtensors")) {
      if (json["postprocess"]["outputtensors"].HasMember("segment")) {
        segment_index_ = json["postprocess"]["outputtensors"]["segment"].GetInt();
      }
    }

    std::string segment_type;
    if (json["postprocess"].HasMember("segment_type")) {
      segment_type = json["postprocess"]["segment_type"].GetString();
      if (segment_type == "S8") {
        seg_type_ = seg_Type::S8;
      } else if (segment_type == "S32") {
        seg_type_ = seg_Type::S32;
      } else if (segment_type == "S64") {
        seg_type_ = seg_Type::S64;
      }
    }
  }

  return 0;
}

void SegmentPostProcess::Parse(std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::shared_ptr<HobotBevData>& result) {
  if (segment_index_ < 0) {
    return;
  }

  if (!result) {
    RCLCPP_ERROR(rclcpp::get_logger("bev_det_seg"),
                "Invalid dnn parser result!");
    return;
  }

  int8_t *cls_data_8;
  int64_t *cls_data_64;
  switch (seg_type_) {
    case seg_Type::S8:
      cls_data_8 = reinterpret_cast<int8_t *>(
          tensors[segment_index_]->sysMem[0].virAddr);
      break;
    case seg_Type::S64:
      cls_data_64 = reinterpret_cast<int64_t *>(
          tensors[segment_index_]->sysMem[0].virAddr);
      break;
    default:
      break;
  }
  auto shift = tensors[segment_index_]->properties.shift.shiftData;
  int shape_h, shape_w, shape_c;
  int aligned_shape_h, aligned_shape_w, aligned_shape_c;
  switch (tensors[segment_index_]->properties.tensorLayout) {
    case HB_DNN_LAYOUT_NHWC: {
      shape_h = tensors[segment_index_]->properties.validShape.dimensionSize[1];
      shape_w = tensors[segment_index_]->properties.validShape.dimensionSize[2];
      shape_c = tensors[segment_index_]->properties.validShape.dimensionSize[3];
      aligned_shape_h =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[1];
      aligned_shape_w =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[2];
      aligned_shape_c =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[3];
      break;
    }
    case HB_DNN_LAYOUT_NCHW: {
      shape_c = tensors[segment_index_]->properties.validShape.dimensionSize[1];
      shape_h = tensors[segment_index_]->properties.validShape.dimensionSize[2];
      shape_w = tensors[segment_index_]->properties.validShape.dimensionSize[3];
      aligned_shape_c =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[1];
      aligned_shape_h =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[2];
      aligned_shape_w =
          tensors[segment_index_]->properties.alignedShape.dimensionSize[3];
      break;
    }
    default:
      printf("not support output layout\n");
  }

#if 0
  std::stringstream ss;
  ss << shape_h << " " << shape_w << "\n";
  int mask_value_idx = 0;
  for (int h = 0; h < shape_h; h++) {
    for (int w = 0; w < shape_w; w++) {
      mask_value_idx = h * aligned_shape_w + w;
      auto v = cls_data_8[mask_value_idx];
      ss << h << " " << w << " ";
      if (seg_type_ == seg_Type::S8) {
        ss << float(cls_data_8[mask_value_idx]);
      } else if (seg_type_ == seg_Type::S64) {
        ss << float(cls_data_64[mask_value_idx]);
      } else {
        ss << "do not support type!";
      }
      ss << "\n";
    }
  }
  // printf("%s\n", ss.str().data());
  std::ofstream ofs("ofs_seg_tros.txt");
  ofs << ss.str();
#endif


  auto& seg = result->seg;
  float* data = reinterpret_cast<float*>(tensors[0]->sysMem[0].virAddr);
  seg.data.resize(shape_w * shape_h);
  seg.valid_w = shape_w;
  seg.valid_h = shape_h;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("bev_det_seg"),
              "shape_w: " << shape_w
              << ", shape_h " << shape_w
              << ", aligned_shape_w: " << aligned_shape_w
              << ", aligned_shape_h： " << aligned_shape_h);

  int mask_value_idx = 0;
  int data_val_idx = 0;
  for (int h = 0; h < shape_h; h++) {
    for (int w = 0; w < shape_w; w++) {
      mask_value_idx = h * aligned_shape_w + w;
      auto v = cls_data_8[mask_value_idx];
      if (v < 0 || v > 4) {
        RCLCPP_ERROR(rclcpp::get_logger("bev_det_seg"),
                    "error seg value: %d", v);
      }

      if (seg_type_ == seg_Type::S8) {
        seg.data[data_val_idx] = cls_data_8[mask_value_idx];
      } else if (seg_type_ == seg_Type::S64) {
        seg.data[data_val_idx] = cls_data_64[mask_value_idx];
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("bev_det_seg"),
                    "do not support type!");
      }
      data_val_idx++;
    }
  }

}
void SegmentPostProcess::ArgMaxChannel(const int &channel,
                                       const std::vector<float> &vec_data_seg,
                                       std::vector<int> &vec_filter_seg) {
  if (vec_data_seg.empty()) {
    return;
  }
  int size = vec_data_seg.size() / channel;
  for (int i = 0; i < size; i++) {
    int max_p = max_element(vec_data_seg.begin() + channel * i,
                            vec_data_seg.begin() + channel * i + channel) -
                vec_data_seg.begin();
    vec_filter_seg.emplace_back(max_p % channel);
  }
}
void SegmentPostProcess::ConvertOutputNCHW(
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

  for (int hh = 0; hh < real_shape.dimensionSize[2]; hh++) {
    for (int ww = 0; ww < real_shape.dimensionSize[3]; ++ww) {
      for (int cc = 0; cc < real_shape.dimensionSize[1]; ++cc) {
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
}  // namespace bev
}  // namespace hobot
