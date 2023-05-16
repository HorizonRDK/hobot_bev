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

#ifndef BEV_INCLUDE_BEV_BEV_NODE_H_
#define BEV_INCLUDE_BEV_BEV_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "jsonutil.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "preprocess.h"
#include "postprocess/postprocess.h"
#include "render.h"

namespace hobot {
namespace bev {

struct BevNodeOutput : public hobot::dnn_node::DnnNodeOutput {
  // size is 6 and seq is:
  // image_front_left
  // image_front
  // image_front_right
  // image_back_left
  // image_back
  // image_back_right
  std::vector<std::string> image_files;
};

class BevNode : public hobot::dnn_node::DnnNode {
 public:
  BevNode(const std::string& node_name = "bev_node",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  // 实现基类的纯虚接口，用于配置Node参数
  int SetNodePara() override;
  // 实现基类的虚接口，将解析后结构化的算法输出数据封装成ROS Msg后发布
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>&
                      node_output) override;

 private:
  void RunSingleFeedInfer();
  void RunBatchFeedInfer();

  std::string config_file_ = "config/bev_ipm_base/bev_ipm_base_config.json";
  std::string model_file_ = "config/model/model-c359f50c.hbm";
  std::string pkg_path_ = ".";

  // single feed imgs
  std::vector<std::string> image_files{
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_FRONT_LEFT__1535639705754799.jpg",
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_FRONT__1535639706262404.jpg",
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_FRONT_RIGHT__1535639706770482.jpg",
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_BACK_LEFT__1535639706797405.jpg",
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_BACK__1535639706787558.jpg",
    "config/bev_ipm_base/bev_test_imgs/n008-2018-08-30-10-33-52-0400__CAM_BACK_RIGHT__1535639706778113.jpg"
  };

  // batch feed img lists
  // The image path = image_pre_path_ + path in list
  std::string image_pre_path_ = "./data/";
  std::vector<std::string> image_lists{
    "config/bev_ipm_base/bev_test_img_lists/cam_front_left.list",  
    "config/bev_ipm_base/bev_test_img_lists/cam_front.list",
    "config/bev_ipm_base/bev_test_img_lists/cam_front_right.list",
    "config/bev_ipm_base/bev_test_img_lists/cam_back_left.list", 
    "config/bev_ipm_base/bev_test_img_lists/cam_back.list",      
    "config/bev_ipm_base/bev_test_img_lists/cam_back_right.list"  
  };

  std::shared_ptr<PreProcess> sp_preprocess_ = nullptr;
  std::shared_ptr<BevPostProcess> sp_postprocess_ = nullptr;
  
  // 算法推理结果消息发布者
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;
  //用于ros方式发布图片
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_publisher_ =
      nullptr;
  std::string msg_pub_topic_name_ = "/image_jpeg";

  std::shared_ptr<BevRender> sp_bev_render_ = nullptr;

  int model_in_img_size_ = 6;
};  // class BevNode

}  // namespace bev
}  // namespace hobot

#endif  // BEV_INCLUDE_BEV_BEV_NODE_H_
