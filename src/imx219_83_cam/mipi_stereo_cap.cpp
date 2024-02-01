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

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "imx219_83_cam/mipi_stereo_cap.h"

namespace mipi_stereo_cam
{
  
MipiStereoCap::MipiStereoCap(int width, int height, std::vector<int> video_indexes) {
  if (video_indexes.empty()) {
    return;
  }

  video_indexes_ = video_indexes;
  parms_.fps = -1;
  parms_.raw_height = height;
  parms_.raw_width = width;

  for (const auto& video_index : video_indexes_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("MipiStereoCap"),
      "width: " << width << ", height: " << height << ", video_index: " << video_index);
            
    auto camera = sp_init_vio_module();
    if (!camera)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("MipiStereoCap"),
        "video_index: " << video_index << " sp_open_camera_v2 failed");
      rclcpp::shutdown();
      return;
    }

    int ret = sp_open_camera_v2(camera, video_index, video_index, 1, &parms_, &parms_.raw_width, &parms_.raw_height);
    if (ret != 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("MipiStereoCap"),
        "video_index: " << video_index << " sp_open_camera_v2 failed");
      rclcpp::shutdown();
      return;
    }

    cameras_.emplace_back(camera);
    
    RCLCPP_WARN_STREAM(rclcpp::get_logger("MipiStereoCap"),
        "video_index: " << video_index << " sp_open_camera_v2 success");
  }
  
  // sleep(2); // wait for isp stability
}


MipiStereoCap::~MipiStereoCap() {
  for (auto& cam : cameras_) {
    sp_vio_close(cam);
    sp_release_vio_module(cam);
  }
}

std::vector<std::shared_ptr<MipiStereoCamImg>> MipiStereoCap::GetImg() {
  std::vector<std::shared_ptr<MipiStereoCamImg>> imgs;
  if (cameras_.size() != video_indexes_.size()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MipiStereoCap"),
        "img size: " << imgs.size() << " is unmatch with video_indexes_ size: " << video_indexes_.size());
    return imgs;
  }
  
#if 0
  for (size_t idx = 0; idx < cameras_.size() - 1; idx++) {
    auto video_index = video_indexes_.at(idx);
    int yuv_size = parms_.raw_width * parms_.raw_height * 1.5;
    static char *yuv_data = new char[yuv_size * sizeof(char)];

    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "start idx: " << video_index << ", count_: " << count_);
      rclcpp::Time ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cameras_.at(idx), yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
        continue;
      }
      rclcpp::Time ts_end = rclcpp::Clock().now();
      auto time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "done idx: " << video_index << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    }

    {
      idx++;
      video_index = video_indexes_.at(idx);
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "start idx: " << video_index << ", count_: " << count_);
      rclcpp::Time ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cameras_.at(idx), yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
        continue;
      }
      rclcpp::Time ts_end = rclcpp::Clock().now();
      auto time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "done idx: " << video_index << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    }

    count_++;
    
    // imgs.push_back(std::shared_ptr<MipiStereoCamImg>(new MipiStereoCamImg(yuv_data, parms_.raw_width, parms_.raw_height, video_index),
    //     [](MipiStereoCamImg* img){
    //       if (img) {
    //         if (img->data_) {
    //           delete [](img->data_);
    //           img->data_ = nullptr;
    //         }

    //         delete img;
    //         img = nullptr;
    //       }
    //     }));
  }
#endif

#if 0
  size_t idx = 0;
  {
    auto video_index = video_indexes_.at(idx);
    int yuv_size = parms_.raw_width * parms_.raw_height * 1.5;
    char *yuv_data = new char[yuv_size * sizeof(char)];

    {
      auto cam = cameras_.at(idx);
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "start idx: " << video_index << ", count_: " << count_);
      rclcpp::Time ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cam, yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
      }
      rclcpp::Time ts_end = rclcpp::Clock().now();
      auto time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();
      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "done idx: " << 0 << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    // }

    // {
      // rclcpp::Time 
      ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cameras_.at(1), yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
      }
      // rclcpp::Time 
      ts_end = rclcpp::Clock().now();

      // auto 
      time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();

      RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                        "done idx: " << 1 << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    }
  }
#endif

#if 1
  rclcpp::Time ts = rclcpp::Clock().now();
  for (size_t idx = 0; idx < cameras_.size(); idx++) {
    auto video_index = video_indexes_.at(idx);
    int yuv_size = parms_.raw_width * parms_.raw_height * 1.5;

    {
      char *yuv_data = new char[yuv_size * sizeof(char)];
      // RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
      //                   "start idx: " << video_index << ", count_: " << count_);
      rclcpp::Time ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cameras_.at(idx), yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
        continue;
      }
      rclcpp::Time ts_end = rclcpp::Clock().now();
      auto time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
      //                   "done idx: " << video_index << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    
      imgs.push_back(std::shared_ptr<MipiStereoCamImg>(new MipiStereoCamImg(yuv_data, parms_.raw_width, parms_.raw_height, video_index, ts),
          [](MipiStereoCamImg* img){
            if (img) {
              if (img->data_) {
                delete [](img->data_);
                img->data_ = nullptr;
              }

              delete img;
              img = nullptr;
            }
          }));
    }

    {
      char *yuv_data = new char[yuv_size * sizeof(char)];
      idx++;
      video_index = video_indexes_.at(idx);
      // RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
      //                   "start idx: " << video_index << ", count_: " << count_);
      rclcpp::Time ts_start = rclcpp::Clock().now();
      if (sp_vio_get_yuv(cameras_.at(idx), yuv_data, parms_.raw_width, parms_.raw_height, 2000) < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
                          "video idx: " << video_index << " sp_vio_get_yuv fail");
        continue;
      }
      rclcpp::Time ts_end = rclcpp::Clock().now();
      auto time_ns = ts_end.nanoseconds() - ts_start.nanoseconds();
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_stereo_cam_node"),
      //                   "done idx: " << video_index << ", count_: " << count_ << ", time ms: " << time_ns / 1000 / 1000);
    
      imgs.push_back(std::shared_ptr<MipiStereoCamImg>(new MipiStereoCamImg(yuv_data, parms_.raw_width, parms_.raw_height, video_index, ts),
          [](MipiStereoCamImg* img){
            if (img) {
              if (img->data_) {
                delete [](img->data_);
                img->data_ = nullptr;
              }

              delete img;
              img = nullptr;
            }
          }));
    }

  }

  count_++;
  
#endif

  if (imgs.size() != cameras_.size()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MipiStereoCap"),
        "img size: " << imgs.size() << " is unmatch with cam size: " << cameras_.size());
    imgs.resize(0);
  }

  return imgs;
}

}
