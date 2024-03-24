[English](./README.md) | 简体中文

# 功能介绍

BEV感知算法是使用地平线[OpenExplorer](https://developer.horizon.ai/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/bev.html)在[nuscenes](https://www.nuscenes.org/nuscenes)数据集上训练出来的`BEV`多任务模型。

算法输入为6组图像数据，分别是前视，左前，右前，后视，左后，右后图。模型输出为10个类别的目标以及对应的3D检测框，包括障碍物、多种类型车辆、交通标志等，以及车道线、人行道、马路边缘的语义分割。

此示例使用本地图像数据作为输入，利用BPU进行算法推理，发布算法感知结果渲染的图片消息，在PC端浏览器上渲染显示算法结果。

# 物料清单


# 使用方法

## 功能安装

在RDK系统的终端中运行如下指令，即可快速安装：

```bash
sudo apt update
sudo apt install -y tros-hobot-bev
sudo apt install -y tros-websocket
```

## 准备回灌数据集

在RDK系统的终端中运行如下指令，下载并解压数据集：

```shell
# 板端下载数据集
wget http://sunrise.horizon.cc/TogetheROS/data/hobot_bev_data.tar.gz

# 解压缩
mkdir -p hobot_bev_data
tar -zxvf hobot_bev_data.tar.gz -C hobot_bev_data

# 解压完成后数据集在hobot_bev_data/data路径下
```

## 启动算法和图像可视化

在RDK系统的终端中运行如下指令，启动算法和可视化：

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动websocket服务
ros2 launch websocket websocket_service.launch.py

# 启动运行脚本，并指定数据集路径
ros2 launch hobot_bev hobot_bev.launch.py image_pre_path:=hobot_bev_data/data
```

启动成功后，打开同一网络电脑的浏览器，访问RDK的IP地址http://IP:8000（IP为RDK的IP地址），即可看到算法可视化的实时效果：

![bev](img/bev.gif)


# 接口说明

## 话题

| 名称         | 消息类型                             | 说明                                     |
| ------------ | ------------------------------------ | ---------------------------------------- |
| /image_jpeg  | sensor_msgs/msg/Image                | 周期发布的图像话题，jpeg格式             |

## 参数

| 名称                         | 参数值                                          | 说明                                               |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| image_pre_path                 | 使用回灌数据集实际所在路径 | 回灌数据集路径                         |


# 常见问题
