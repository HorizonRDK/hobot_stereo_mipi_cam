# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(stereo_mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-w)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(
  include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
)

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mipi_stereo_cam_node.cpp
  src/imx219_83_cam/mipi_stereo_cap.cpp
)

set(BASE_LIBRARIES pthread vio cam dl)
set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin)
set(X3_C_DEV spcdev m hbspdev)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}  ${X3_C_DEV} yaml-cpp opencv_world
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
  )

if(BUILD_HBMEM)
  target_link_libraries(${PROJECT_NAME}
    hbmem
  )
  ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    hbm_img_msgs
  )
endif()

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
