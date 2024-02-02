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

#ifndef MIPI_STEREO_CAM_NODE_H_
#define MIPI_STEREO_CAM_NODE_H_
#include "mipi_stereo_cam_node.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/string.hpp>

#include "imx219_83_cam/mipi_stereo_cap.h"

namespace mipi_stereo_cam
{
class MipiStereoCamNode : public rclcpp::Node
{
public:
  MipiStereoCamNode(const std::string& node_name);
  ~MipiStereoCamNode();

private:
  int GetParams();
  int CheckParams();

  int Init();
  
  // <index, image>
  void OnRecvedImg(std::pair<uint64_t, std::vector<std::shared_ptr<MipiStereoCamImg>>> imgs);

  // For non-blocking keyboard inputs
  int Getch();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;

  std::shared_ptr<MipiStereoCap> sp_cam_cap_ = nullptr;
  std::shared_ptr<std::thread> sp_getimg_task_ = nullptr;

  std::shared_ptr<std::thread> sp_work_ = nullptr;
  std::shared_ptr<std::thread> sp_teleop_task_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ros_compressed_image_publisher_ = nullptr;

  // parameters
  std::string frame_id_;
  std::string io_method_ = "ros";  // hbmem zero mem copy
  std::string imgMsgPtr;
  int image_width_ = 1920;
  int image_height_ = 1080;
  std::string out_format_ = "nv12";
  std::atomic_bool is_init_;

  std::string camera_name_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;

  size_t cache_len_limit_ = 10;
  std::vector<int> video_index_ {0, 2};
  std::map<int, uint64_t> counts_;
  uint64_t get_img_counts_ = 0;

  std::map<int, std::queue<std::shared_ptr<MipiStereoCamImg>>> img_cache_;
  std::mutex img_mtx_;

  std::queue<std::function<void()>> img_process_task_cache_;
  std::mutex img_process_task_mtx_;
  std::condition_variable img_process_task_cv_;

  int data_sampling_rate_ = -1;
  uint64_t dump_count_ = 0;
  std::atomic_bool enable_dump_;
};
}  // namespace mipi_stereo_cam
#endif  // MIPI_STEREO_CAM_NODE_H_
