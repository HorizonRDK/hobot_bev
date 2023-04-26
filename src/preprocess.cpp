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
      sp_feedback_data->image_files.size() + sp_feedback_data->points_files.size() != pmodel->GetInputCount()) {
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

    auto input = GetNV12Pyramid(image_file, in_h, in_w);
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

std::shared_ptr<DNNTensor> PreProcess::GetNV12Pyramid(const std::string &image_file,
                                                      int scaled_img_height,
                                                      int scaled_img_width) {
  int original_img_height = 0, original_img_width = 0;
  return GetNV12Pyramid(image_file, scaled_img_height, scaled_img_width,
                        original_img_height, original_img_width);
}

std::shared_ptr<DNNTensor> PreProcess::GetNV12Pyramid(const std::string &image_file,
                                                      int scaled_img_height,
                                                      int scaled_img_width,
                                                      int &original_img_height,
                                                      int &original_img_width) {
  cv::Mat nv12_mat;
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  cv::Mat mat_tmp;
  mat_tmp.create(scaled_img_height, scaled_img_width, bgr_mat.type());
  cv::resize(bgr_mat, mat_tmp, mat_tmp.size());
  auto ret = PreProcess::BGRToNv12(mat_tmp, nv12_mat);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }
  original_img_height = bgr_mat.rows;
  original_img_width = bgr_mat.cols;

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  // padding y
  for (int h = 0; h < scaled_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    for (int w = 0; w < scaled_img_width; ++w) {
      *raw++ = *data++;
    }
  }

  // padding uv
  auto uv_data = nv12_mat.data + scaled_img_height * scaled_img_width;
  for (int32_t h = 0; h < scaled_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    for (int32_t w = 0; w < scaled_img_width; ++w) {
      *raw++ = *uv_data++;
    }
  }

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;
  input_tensor->sysMem[0].virAddr = reinterpret_cast<void *>(y->virAddr);
  input_tensor->sysMem[0].phyAddr = y->phyAddr;
  input_tensor->sysMem[0].memSize = scaled_img_height * scaled_img_width;
  input_tensor->sysMem[1].virAddr = reinterpret_cast<void *>(uv->virAddr);
  input_tensor->sysMem[1].phyAddr = uv->phyAddr;
  input_tensor->sysMem[1].memSize = scaled_img_height * scaled_img_width / 2;
  // auto pyramid = new NV12PyramidInput;
  // pyramid->width = scaled_img_width;
  // pyramid->height = scaled_img_height;
  // pyramid->y_vir_addr = y->virAddr;
  // pyramid->y_phy_addr = y->phyAddr;
  // pyramid->y_stride = w_stride;
  // pyramid->uv_vir_addr = uv->virAddr;
  // pyramid->uv_phy_addr = uv->phyAddr;
  // pyramid->uv_stride = w_stride;
  return std::shared_ptr<DNNTensor>(
      input_tensor, [y, uv](DNNTensor *input_tensor) {
        // Release memory after deletion
        std::cout << "Release input_tensor" << std::endl;
        hbSysFreeMem(y);
        hbSysFreeMem(uv);
        delete y;
        delete uv;
        delete input_tensor;
      });
}

int32_t PreProcess::BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
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
