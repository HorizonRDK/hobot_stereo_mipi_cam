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
    : is_init_(false), enable_dump_(false), Node(node_name, rclcpp::NodeOptions()) {
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
  dump_img_task_cv_.notify_all();
  if (sp_dumptask_) {
    sp_dumptask_->join();
  }
  RCLCPP_WARN(rclcpp::get_logger("mipi_stereo_cam_node"), "shutting down");
}

int MipiStereoCamNode::GetParams() {
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("framerate", framerate_);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", image_height_);
  this->declare_parameter("image_width", image_width_);
  this->declare_parameter("io_method", io_method_);
  this->declare_parameter("out_format", out_format_);
  this->declare_parameter("video_device", video_device_);
  this->declare_parameter("data_sampling_rate", data_sampling_rate_);
  
  this->get_parameter<std::string>("camera_name", camera_name_);
  this->get_parameter<int>("framerate", framerate_);
  this->get_parameter<int>("image_height", image_height_);
  this->get_parameter<int>("image_width", image_width_);
  this->get_parameter<std::string>("io_method", io_method_);
  this->get_parameter<std::string>("out_format", out_format_);
  this->get_parameter<std::string>("video_device", video_device_);
  this->get_parameter<int>("data_sampling_rate", data_sampling_rate_);

  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("mipi_stereo_cam_node"),
      "Get params complete."
      << "\n camera_name: " << camera_name_
      << "\n video_device_name: " << video_device_
      << "\n image_width: " << image_width_
      << "\n image_height: " << image_height_
      << "\n io_method_name: " << io_method_
      << "\n out_format: " << out_format_
      << "\n data_sampling_rate: " << data_sampling_rate_
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

  if (io_method_.compare("ros") == 0) {
    image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
  } else {
#ifdef USING_HBMEM
    // 创建hbmempub
    publisher_hbmem_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            "hbmem_img", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiStereoCamNode::hbmem_update, this));
#else
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                      "Hbmem is not enabled.");
#endif
  }
  

  // 创建cam实例
  sp_cam_cap_ = std::make_shared<MipiStereoCap>(1920, 1080, video_index_);
  if (!sp_cam_cap_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                      "Invalid MipiStereoCap instance!");
    rclcpp::shutdown();
    return -1;
  }

  if (data_sampling_rate_ > 0) {
    // Data collecting is enabled
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
    
    // 图像dump任务
    sp_dumptask_ = std::make_shared<std::thread>([this](){
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(dump_img_task_mtx_);
        dump_img_task_cv_.wait(lg, [this](){
          return !rclcpp::ok() || !dump_img_task_cache_.empty();
        });
        if (!rclcpp::ok()) {
          break;
        }
        if (dump_img_task_cache_.empty()) {
          continue;
        }
        
        auto task = dump_img_task_cache_.front();
        dump_img_task_cache_.pop();
        lg.unlock();

        task();
      }
    });

    // 获取键盘输入任务
    sp_teleop_task_ = std::make_shared<std::thread>([this](){
      while (rclcpp::ok()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "waiting for key...\n"
          << "g/G: dump one group imgs \n"
          << "q/Q: stop and exit\n");
          
        char key = Getch();
        // printf("command: %c\n\n", key);
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "capture key: " << key);

        //'A' and 'B' represent the Up and Down arrow keys consecutively 
        if (key=='q'||key=='Q') {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "exit...");
          enable_dump_ = false;
          rclcpp::shutdown();
          return;
        } else if (key=='g'||key=='G') {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "enable dump...");
          enable_dump_ = true; 
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"), "capture invalid key: " << key
            << ", valid keys: \n"
            << "g/G: dump one group imgs \n"
            << "q/Q: stop and exit\n");
        }
      }
    });
  }

  // 创建图像获取任务
  auto get_img = [this](){
    {
      std::vector<std::shared_ptr<MipiStereoCamImg>> imgs = sp_cam_cap_->GetImg();
      // continue;
      if (enable_dump_) {
        enable_dump_ = false;
        
        std::lock_guard<std::mutex> lg(dump_img_task_mtx_);
        if (dump_img_task_cache_.size() > cache_len_limit_) {
          dump_img_task_cache_.pop();
        }
        
        dump_img_task_cache_.push(std::bind(&MipiStereoCamNode::DumpTask, this,
          std::pair<int, std::vector<std::shared_ptr<MipiStereoCamImg>>>(dump_count_++, imgs)));
        dump_img_task_cv_.notify_one();
      }
      return;

      // 检查是否需要dump
      if (data_sampling_rate_ > 0) {
        // Data collecting is enabled
        get_img_counts_++;
        if (get_img_counts_ >= data_sampling_rate_) {
          get_img_counts_ = 0;

          std::lock_guard<std::mutex> lg(dump_img_task_mtx_);
          if (dump_img_task_cache_.size() > cache_len_limit_) {
            dump_img_task_cache_.pop();
          }
          
          dump_img_task_cache_.push(std::bind(&MipiStereoCamNode::DumpTask, this,
            std::pair<int, std::vector<std::shared_ptr<MipiStereoCamImg>>>(dump_count_++, imgs)));
          dump_img_task_cv_.notify_one();
        }
      }
    }
  };


  sp_getimg_task_ = std::make_shared<std::thread>([this, get_img](){
    while (rclcpp::ok()) {
      get_img();
    }
  });
  std::cout << "\n\n sp_getimg_task_ id: " << sp_getimg_task_->get_id() << "\n\n";

  // timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(static_cast<int64_t>(10)), get_img);

  is_init_ = true;
  return 0;
}

void MipiStereoCamNode::GetImg() {

  rclcpp::Time t;

  sensor_msgs::msg::Image::UniquePtr imgMsgPtr = std::make_unique<sensor_msgs::msg::Image>();
  imgMsgPtr->header.stamp = t;

  int num = 1;  // for endianness detection
  imgMsgPtr->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  size_t size = imgMsgPtr->step * imgMsgPtr->height;
  imgMsgPtr->data.resize(size);

  int dataSize = 0;
  char * srcL;
  char * srcR;

  imgMsgPtr->header.frame_id = frame_id_;
  image_pub_->publish(*imgMsgPtr);
}

void MipiStereoCamNode::hbmem_update() {
#ifdef USING_HBMEM
  if (cam_cap_.is_capturing()) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!cam_cap_.get_image_mem(msg.time_stamp,
                                  msg.encoding,
                                  msg.height,
                                  msg.width,
                                  msg.step,
                                  msg.data,
                                  msg.data_size)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_stereo_cam_node"),
                     "hbmem_update grab img failed");
        return;
      }
      msg.index = mSendIdx++;
      publisher_hbmem_->publish(std::move(loanedMsg));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_stereo_cam_node"),
                  "borrow_loaned_message failed");
    }
  }
#endif
}

void MipiStereoCamNode::DumpTask(std::pair<uint64_t, std::vector<std::shared_ptr<MipiStereoCamImg>>> img_pair) {
  RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
    "Dump task");
  for (const auto& sp_img : img_pair.second) {
    if (!sp_img) {
      continue;
    }

    // nv12 to jpg
    cv::Mat nv12(sp_img->h_ * 3 / 2, sp_img->w_, CV_8UC1, sp_img->data_);
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);

    if (sp_img->w_ != image_width_ || sp_img->h_ != image_height_) {
      // 720P
      cv::Mat mat_tmp;
      mat_tmp.create(image_height_, image_width_, bgr.type());
      cv::resize(bgr, mat_tmp, mat_tmp.size(), 0, 0);

      std::string str_idx = std::to_string(img_pair.first);
      if (str_idx.length() < 7) {
        str_idx = std::string(7 - str_idx.length(), '0') + str_idx;
      }
      
      std::string file_name =
        "./cam_" + std::to_string(sp_img->video_index_) + "/" +
        str_idx + ".jpg";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
        "Dump data collecting file: " << file_name);
      cv::imwrite(file_name, mat_tmp);
    }
  }
}

int MipiStereoCamNode::Getch() {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

}  // namespace mipi_stereo_cam
