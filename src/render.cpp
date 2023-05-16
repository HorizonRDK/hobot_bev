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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "render.h"

namespace hobot {
namespace bev {

int BevRender::Render(
    const std::vector<std::string>& image_files,
    const std::shared_ptr<HobotBevData>& det_result,
    cv::Mat& mat_bg) {
  if (image_files.empty() || !det_result) return -1;

  int scaled_img_width = std::max(render_para.ipm_seg_size_w, render_para.ipm_bev_size_w);
  int scaled_img_height = std::max(render_para.ipm_seg_size_h, render_para.ipm_bev_size_h);

  mat_bg = cv::Mat(scaled_img_height * render_para.layout_h_num,
    scaled_img_width * render_para.layout_w_num, CV_8UC3, cv::Scalar::all(0));

  int copy_offset_w = 0;
  int copy_offset_h = 0;

  for (size_t idx = 0; idx < image_files.size(); idx++) {
    const auto& image_file = image_files.at(idx);
    if (access(image_file.c_str(), F_OK) != 0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_bev"), "File is not exist! image_file: " << image_file);
      return -1;
    }

    cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
    cv::Mat mat_tmp;
    cv::resize(bgr_mat, mat_tmp, cv::Size(scaled_img_width, scaled_img_height));

    // render img info
    if (idx < render_para.image_infos.size()) {
      cv::putText(mat_tmp,
                  render_para.image_infos.at(idx),
                  cv::Point2f(10, scaled_img_height - 10),
                  cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  cv::Scalar(255, 255, 255),
                  1.5);
     }

    mat_tmp.copyTo(mat_bg(cv::Rect(copy_offset_w, copy_offset_h,
                                  mat_tmp.cols,
                                  mat_tmp.rows)));
    copy_offset_w += scaled_img_width;
    if (2 == idx || 5 == idx) {
      copy_offset_w = 0;
      copy_offset_h += scaled_img_height;
    }
  }
    
  // {
  //   std::string saving_path = "render_bg.jpeg";
  //   RCLCPP_WARN(rclcpp::get_logger("BevRender"),
  //               "Draw result to file: %s",
  //               saving_path.c_str());
  //   cv::imwrite(saving_path, mat_bg);
  // }
  
  // render box
  cv::Mat mat_bgr_box(render_para.ipm_bev_size_h, render_para.ipm_bev_size_w, CV_8UC3, cv::Scalar(255, 255, 255));
  {
    RCLCPP_INFO(rclcpp::get_logger("BevRender"),
                "det_result->bbox3ds.size: %d", det_result->bbox3ds.size());

    for (const auto& bbox3d : det_result->bbox3ds) {
      RCLCPP_INFO(rclcpp::get_logger("BevRender"),
                  "bbox3d.size: %d", bbox3d.size());
      for (const auto &det : bbox3d) {
        cv::Scalar color = cv::Scalar(255, 0, 0);
        auto rect_colors_iter = render_para.rect_colors.find(det.cls_str);
        if (render_para.rect_colors.end() != rect_colors_iter) {
          color = rect_colors_iter->second;
        }
        
        float center_w, center_h;
        center_w = det.l / render_para.ipm_x_meter_per_pixel;
        center_h = det.h / render_para.ipm_y_meter_per_pixel;
        
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("BevRender"),
        //             "center_h " << center_h
        //             << " det.h " << det.h
        //             << " ipm_y_meter_per_pixel " << render_para.ipm_y_meter_per_pixel);
        auto center_x = render_para.ipm_bev_size_w / 2 - det.y / render_para.ipm_y_meter_per_pixel;
        auto center_y = render_para.ipm_bev_size_h / 2 - det.x / render_para.ipm_x_meter_per_pixel;
        cv::RotatedRect rect =
            cv::RotatedRect(cv::Point2f(det.x, det.y),
                            cv::Size2f(det.l, det.h), det.r);
        cv::Point2f arr[4];
        rect.points(arr);
        auto box_type = det.cls_str;
        for (int i = 0; i < 4; i++) {
          cv::line(mat_bgr_box, arr[i], arr[(i + 1) % 4], color, 1, CV_AA);
        }
        
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("BevRender"),
                    "Draw cls_str: " << det.cls_str
                    << " det.x " << det.x << " y " << det.y << " l " << det.l << " h " << det.h
                    << " center_x " << center_x << " y " << center_y << " w " << center_w << " h " << center_h);
      }
    }

    // render car self
    cv::Scalar color_car = cv::Scalar(0, 0, 0);
    int x1, y1, x2, y2;
    x1 = render_para.ipm_car_x - render_para.car_width / 2;
    y1 = render_para.ipm_car_y - render_para.car_heigth / 2;
    x2 = render_para.ipm_car_x + render_para.car_width / 2;
    y2 = render_para.ipm_car_y + render_para.car_heigth / 2;
    cv::rectangle(mat_bgr_box, cv::Point(x1, y1), cv::Point(x2, y2), color_car, cv::FILLED);

    // std::string saving_path = "render_box.jpeg";
    // RCLCPP_WARN(rclcpp::get_logger("BevRender"),
    //             "Draw result to file: %s",
    //             saving_path.c_str());
    // cv::imwrite(saving_path, mat_bgr_box);
  }

  // 逆时针旋转90°使检测框渲染图片的上方是车辆前进方向
  // 顺时针90°旋转: transpose + flip(tmp,dst,1)
  // 逆时针90°旋转:transpose + flip(tmp,dst,0)
  // 180°旋转: flip(src,dst,-1)
	cv::Mat temp;
	cv::transpose(mat_bgr_box, temp);
	cv::flip(temp, mat_bgr_box, 0);

  // render seg
  cv::Mat mat_bgr_seg(render_para.ipm_seg_size_h, render_para.ipm_seg_size_w, CV_8UC3, cv::Scalar(255, 255, 255));
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("BevRender"),
                "ipm_seg_size_h: " << render_para.ipm_seg_size_h
                << ", w: " << render_para.ipm_seg_size_w
                << ", perception h: " << det_result->seg.valid_h
                << ", w: " << det_result->seg.valid_w);

    int mat_height = det_result->seg.valid_h;
    int mat_width = det_result->seg.valid_w;
    cv::Mat mask_img_mat(mat_height, mat_width, CV_8UC3, cv::Scalar(0, 0, 0));
    
    int loopi = 0;
    for (int loopj = 0; loopj < mat_height; loopj++) {
      for (int loopk = 0; loopk < mat_width; loopk++) {
        auto value_itr = det_result->seg.data.at(loopj * mat_width + loopk);
        size_t color_index = static_cast<size_t>(value_itr) % 10;
        if (color_index >= render_para.mask_colors.size()) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("BevRender"),
                      "color_index: " << color_index
                      << " exceeds size: " << render_para.mask_colors.size()
                      );
          continue;
        }

        mask_img_mat.at<cv::Vec3b>(loopj,
                                  loopk + loopi * mat_width)[0] =
            render_para.mask_colors[color_index][0];
        mask_img_mat.at<cv::Vec3b>(loopj,
                                  loopk + loopi * mat_width)[1] =
            render_para.mask_colors[color_index][1];
        mask_img_mat.at<cv::Vec3b>(loopj,
                                  loopk + loopi * mat_width)[2] =
            render_para.mask_colors[color_index][2];
      }
    }
    loopi++;

    // {
    //   std::string saving_path = "render_mask.jpeg";
    //   RCLCPP_WARN(rclcpp::get_logger("BevRender"),
    //               "Draw result to file: %s",
    //               saving_path.c_str());
    //   cv::imwrite(saving_path, mask_img_mat);
    // }
    
    cv::Mat mask_img_mat_resize;
    // sawtooth segment
    if (render_para.sawtooth_pro_) {
      cv::Mat mask_img_mat_resize_tmp;
      cv::Mat median;
      cv::resize(mask_img_mat, mask_img_mat_resize_tmp,
            cv::Size(mat_bgr_seg.cols / 2, mat_bgr_seg.rows / 2));
      
      cv::medianBlur(mask_img_mat_resize_tmp, median, render_para.sawtooth_ksize_);
      cv::resize(median, mask_img_mat_resize, cv::Size(mat_bgr_seg.cols, mat_bgr_seg.rows));
    } else {
      cv::resize(mask_img_mat, mask_img_mat_resize,
            cv::Size(mat_bgr_seg.cols, mat_bgr_seg.rows));
    }
    addWeighted(mat_bgr_seg, render_para.img_Weighted_, mask_img_mat_resize, render_para.segment_Weighted_,
                0.0, mat_bgr_seg);
                
    // std::string saving_path = "render_seg.jpeg";
    // RCLCPP_WARN(rclcpp::get_logger("BevRender"),
    //             "Draw result to file: %s",
    //             saving_path.c_str());
    // cv::imwrite(saving_path, mat_bgr_seg);

  }
  
  // copy box img to bg
  {
    cv::Mat mat_tmp;
    int scaled_img = std::min(scaled_img_height, scaled_img_height);
    mat_tmp.create(scaled_img, scaled_img, mat_bgr_box.type());
    cv::resize(mat_bgr_box, mat_tmp, mat_tmp.size());
    mat_tmp.copyTo(mat_bg(cv::Rect(copy_offset_w, copy_offset_h,
                                  mat_tmp.cols,
                                  mat_tmp.rows)));
    copy_offset_w += scaled_img_width;
  }
  
  // copy seg img to bg
  {
    mat_bgr_seg.copyTo(mat_bg(cv::Rect(copy_offset_w, copy_offset_h,
                                  mat_bgr_seg.cols,
                                  mat_bgr_seg.rows)));
    copy_offset_w += scaled_img_width;
  }

  // render labels on bg
  {
    // rect
    if (!render_para.rect_colors.empty()) {
      int height = scaled_img_height / render_para.rect_colors.size();
      int h_offset = std::min(height * 0.5, 10.0);
      RCLCPP_INFO(rclcpp::get_logger("BevRender"),
                  "rect_colors size: %d, scaled_img_height: %d",
                  render_para.rect_colors.size(), scaled_img_height);
      int pt_x = copy_offset_w + 10;
      int pt_y = copy_offset_h + 5;
      for (const auto& color : render_para.rect_colors) {
        cv::rectangle(mat_bg,
                      cv::Point(pt_x, pt_y),
                      cv::Point(pt_x + 20, pt_y + h_offset),
                      color.second,
                      2
                      );

        cv::putText(mat_bg,
                    color.first,
                    cv::Point2f(pt_x + 20 + 5, pt_y + h_offset),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(255, 255, 255),
                    1.5);
        pt_y += height;
      }
    }

    // mask
    if (render_para.mask_infos.size() == render_para.mask_colors.size()) {
      int height = scaled_img_height / render_para.mask_colors.size();
      int h_offset = std::min(height * 0.5, 10.0);
      RCLCPP_INFO(rclcpp::get_logger("BevRender"),
                  "mask_colors size: %d, scaled_img_height: %d",
                  render_para.mask_colors.size(), scaled_img_height);
      int pt_x = copy_offset_w + scaled_img_width / 2;
      int pt_y = copy_offset_h + 5;
      for (size_t idx = 0; idx < render_para.mask_colors.size(); idx++) {
        const auto& color = render_para.mask_colors.at(idx);
        const auto& info = render_para.mask_infos.at(idx);
        cv::rectangle(mat_bg,
                      cv::Point(pt_x, pt_y),
                      cv::Point(pt_x + 20, pt_y + h_offset),
                      cv::Scalar(color[0], color[1], color[2]),
                      cv::FILLED
                      );
        cv::putText(mat_bg,
                    info,
                    cv::Point2f(pt_x + 20 + 5, pt_y + h_offset),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(255, 255, 255),
                    1.5);
        pt_y += height;
      }
    }
  }

  // static int count = 0;
  // std::string saving_path = "render_bev_" + std::to_string(count++) + ".jpeg";
  // RCLCPP_WARN(rclcpp::get_logger("BevRender"),
  //             "Draw result to file: %s",
  //             saving_path.c_str());
  // cv::imwrite(saving_path, mat_bg);

  return 0;
}

}
}
