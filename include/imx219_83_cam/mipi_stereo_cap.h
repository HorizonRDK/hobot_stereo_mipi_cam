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

#ifndef MIPI_STEREO_CAP_H_
#define MIPI_STEREO_CAP_H_

#include <string>
#include <sp_vio.h>
#include <sp_sys.h>

namespace mipi_stereo_cam
{

struct MipiStereoCamImg
{
  MipiStereoCamImg(char* data, int w, int h, int video_idx, rclcpp::Time ts = rclcpp::Clock().now())
    : data_(data), w_(w), h_(h), video_index_(video_idx), ts_(ts) {}
  char* data_ = nullptr;
  int w_ = 0;
  int h_ = 0;
  int video_index_ = 0;
  rclcpp::Time ts_ = rclcpp::Clock().now();
};

class MipiStereoCap {
public:
  MipiStereoCap(int width, int height, std::vector<int> video_index);
  ~MipiStereoCap();

  std::vector<std::shared_ptr<MipiStereoCamImg>> GetImg();

private:
  sp_sensors_parameters parms_;
  std::vector<void *> cameras_;
  std::vector<int> video_indexes_;
  uint64_t count_ = 0;
};

}

#endif
