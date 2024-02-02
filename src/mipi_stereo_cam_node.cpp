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

#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

#include <stdio.h>
#include <termios.h>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "mipi_stereo_cam_node.h"

namespace mipi_stereo_cam {

MipiStereoCamNode::MipiStereoCamNode(const std::string& node_name)
    : is_init_(false), Node(node_name, rclcpp::NodeOptions()) {
  // Check if camera is already open
  std::string cfg_info;
  std::ifstream sif_info("/sys/devices/platform/soc/a4001000.sif/cfg_info");
  if (sif_info.is_open()) {
    sif_info >> cfg_info;
  }
  // 若mipi camera已打开，sif_info返回“pipeid”
  if (!cfg_info.compare("pipeid")) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_stereo_cam_node"),
                 "mipi camera already in use.\n");
    rclcpp::shutdown();
    return;
  }

  if (GetParams() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "Get params failed!");
    rclcpp::shutdown();
    return;
  }

  if (CheckParams() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "Check params failed!");
    rclcpp::shutdown();
    return;
  }

  if (Init() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "Init failed!");
    rclcpp::shutdown();
    return;
  }
}

MipiStereoCamNode::~MipiStereoCamNode() {
  img_process_task_cv_.notify_all();
  if (sp_work_) {
    sp_work_->join();
  }
  RCLCPP_WARN(rclcpp::get_logger("mipi_stereo_cam_node"), "shutting down");
}

int MipiStereoCamNode::GetParams() {
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", image_height_);
  this->declare_parameter("image_width", image_width_);
  this->declare_parameter("io_method", io_method_);
  this->declare_parameter("out_format", out_format_);
  this->declare_parameter("data_sampling_ms_diff", data_sampling_ms_diff_);
  
  this->get_parameter<std::string>("camera_name", camera_name_);
  this->get_parameter<int>("image_height", image_height_);
  this->get_parameter<int>("image_width", image_width_);
  this->get_parameter<std::string>("io_method", io_method_);
  this->get_parameter<std::string>("out_format", out_format_);
  this->get_parameter<int>("data_sampling_ms_diff", data_sampling_ms_diff_);

  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("mipi_stereo_cam_node"),
      "Get params complete."
      << "\n camera_name: " << camera_name_
      << "\n image_width: " << image_width_
      << "\n image_height: " << image_height_
      << "\n io_method_name: " << io_method_
      << "\n out_format: " << out_format_
      << "\n data_sampling_ms_diff: " << data_sampling_ms_diff_
    );

  return 0;
}

int MipiStereoCamNode::CheckParams() {
  if (io_method_ != "ros" && io_method_ != "shared_mem") {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
      "Invalid input io_method_name: " << io_method_ << ", which should be \"ros\"/\"shared_mem\".");
    return -1;
  }

  return 0;
}

int MipiStereoCamNode::Init() {
  if (is_init_) return 0;

  image_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  ros_compressed_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/image_raw/compressed", 10);

  // 创建cam实例
  sp_cam_cap_ = std::make_shared<MipiStereoCap>(image_width_, image_height_, video_index_);
  if (!sp_cam_cap_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                      "Invalid MipiStereoCap instance!");
    rclcpp::shutdown();
    return -1;
  }

  if (data_sampling_ms_diff_ >= 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                      "Data collecting is enabled");
    for (const auto& video_index : video_index_) {
      std::string path = "./cam_" + std::to_string(video_index);
      if (access(path.data(), W_OK) != 0) {
        std::string cmd = "mkdir -p " + path;
        system(cmd.data());
      }
      
      if (access(path.data(), W_OK) != 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
          "dump data path: " << path << " is not existed!");
        rclcpp::shutdown();
        return -1;
      }
    }
  }

  // 获取到图像后的处理任务
  sp_work_ = std::make_shared<std::thread>([this](){
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lg(img_process_task_mtx_);
      img_process_task_cv_.wait(lg, [this](){
        return !rclcpp::ok() || !img_process_task_cache_.empty();
      });
      if (!rclcpp::ok()) {
        break;
      }
      if (img_process_task_cache_.empty()) {
        continue;
      }
      
      auto task = img_process_task_cache_.front();
      img_process_task_cache_.pop();
      lg.unlock();

      task();
    }
  });
  
  // 创建图像获取任务
  auto get_img = [this](){
    std::vector<std::shared_ptr<MipiStereoCamImg>> imgs = sp_cam_cap_->GetImg();
    std::lock_guard<std::mutex> lg(img_process_task_mtx_);
    if (img_process_task_cache_.size() > cache_len_limit_) {
      img_process_task_cache_.pop();
    }
    
    img_process_task_cache_.push(std::bind(&MipiStereoCamNode::OnRecvedImg, this, imgs));
    img_process_task_cv_.notify_one();
  };

  sp_getimg_task_ = std::make_shared<std::thread>([this, get_img](){
    while (rclcpp::ok()) {
      get_img();
    }
  });

  is_init_ = true;
  return 0;
}

void MipiStereoCamNode::OnRecvedImg(std::vector<std::shared_ptr<MipiStereoCamImg>> img_pair) {
  cv::Mat bgr_stitch;
  int idx = 0;
  rclcpp::Time ts;
  static auto last_collect_tp = std::chrono::system_clock::now();
  auto tp_start = std::chrono::system_clock::now();
  bool do_data_collect = false;
  static uint64_t seq = 0;
  std::string str_idx = std::to_string(seq++);
  if (str_idx.length() < 7) {
    str_idx = std::string(7 - str_idx.length(), '0') + str_idx;
  }

  // 检查是否需要dump
  if (data_sampling_ms_diff_ > 0 &&
    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - last_collect_tp)
                        .count() >= data_sampling_ms_diff_) {
    do_data_collect = true;
    last_collect_tp = std::chrono::system_clock::now();
  }

  for (const auto& sp_img : img_pair) {
    if (!sp_img) {
      continue;
    }
    ts = sp_img->ts_;
    // nv12 to jpg
    cv::Mat nv12(sp_img->h_ * 3 / 2, sp_img->w_, CV_8UC1, sp_img->data_);
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);

    if (bgr_stitch.cols == 0 || bgr_stitch.rows == 0) {
      bgr_stitch.create(bgr.rows, bgr.cols * img_pair.size(), bgr.type());
    }
    bgr.copyTo(bgr_stitch(cv::Rect(bgr.cols * idx,
                                  0,
                                  bgr.cols,
                                  bgr.rows)));
    idx++;
    
    if (do_data_collect) {
      std::string file_name =
        "./cam_" + std::to_string(sp_img->video_index_) + "/" +
        str_idx + ".jpg";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
        "collecting img file: " << file_name);
      cv::imwrite(file_name, bgr);
    }
  }

  // {
  //   static int count = 0;
  //   std::string file_name =
  //     "./" + std::to_string(count++) + ".jpg";
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
  //     "Dump data collecting file: " << file_name);
  //   cv::imwrite(file_name, bgr_stitch);
  // }
  {
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - tp_start)
                        .count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "Image encode time cost " << interval << "ms");
  }

  sensor_msgs::msg::CompressedImage::UniquePtr compressed_img_pub_(new sensor_msgs::msg::CompressedImage());
  compressed_img_pub_->header.stamp = ts;
  compressed_img_pub_->header.frame_id = "default_cam";
  compressed_img_pub_->format = "jpeg";
  
  // 使用opencv的imencode接口将mat转成vector，获取图片size
  std::vector<int> param;
  cv::imencode(".jpg", bgr_stitch, compressed_img_pub_->data, param);

  {
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - tp_start)
                        .count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "imencode processing time cost " << interval << "ms");
  }

  if (ros_compressed_image_publisher_)
    ros_compressed_image_publisher_->publish(*compressed_img_pub_);
}

}  // namespace mipi_stereo_cam
