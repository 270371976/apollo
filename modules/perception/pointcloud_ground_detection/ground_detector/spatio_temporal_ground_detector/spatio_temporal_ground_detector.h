/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "modules/perception/common/algorithm/i_lib/pc/i_ground.h"
#include "modules/perception/common/lidar/scene_manager/ground_service/ground_service.h"
#include "modules/perception/common/lidar/scene_manager/scene_manager.h"
#include "modules/perception/pointcloud_ground_detection/interface/base_ground_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class SpatioTemporalGroundDetector : public BaseGroundDetector {
 public:
  SpatioTemporalGroundDetector() = default;
  ~SpatioTemporalGroundDetector() {
    if (pfdetector_ != nullptr) {
      delete pfdetector_;
    }
    if (param_ != nullptr) {
      delete param_;
    }
  }
  /**
   * @brief Init ground detector
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;
  /**
   * @brief Detect ground point
   *
   * @param options
   * @param frame location to save non_ground_point indices and point height
   * @return true
   * @return false
   */
  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "SpatioTemporalGroundDetector"; }

 private:
  algorithm::PlaneFitGroundDetectorParam* param_ = nullptr;
  algorithm::PlaneFitGroundDetector* pfdetector_ = nullptr;
  std::vector<float> data_;
  std::vector<float> ground_height_signed_;
  std::vector<int> point_indices_temp_;

  bool use_roi_ = true;
  bool use_ground_service_ = false;
  float ground_thres_ = 0.25f;
  size_t default_point_size_ = 320000;
  Eigen::Vector3d cloud_center_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  GroundServiceContent ground_service_content_;
};  // class SpatioTemporalGroundDetector

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
