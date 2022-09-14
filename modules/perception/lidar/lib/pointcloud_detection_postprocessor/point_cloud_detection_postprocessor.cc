/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_detection_preprocessor.h"

namespace apollo {
namespace perception {
namespace lidar {

// stage init
bool PointCloudDetectionPostprocessor::Init(const StageConfig& stage_config) {
  ACHECK(stage_config.has_pointcloud_detection_postprocessor());

  get_objects_ = PluginFactory::CreatePlugin(stage_config.get_objects());
  if (!get_objects_->Init(stage_config.get_objects())) {
    return false;
  }
  return true;
}

bool PointCloudDetectionPostprocessor::Process(DataFrame* data_frame) {
  return true;
}

bool PointCloudDetectionPostprocessor::Process(
    DataFrame* data_frame, std::vector<float>* out_detections,
    std::vector<int>* out_labels) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (nullptr == out_detections) {
    AERROR << "Input null out_detections ptr.";
    return false;
  }

  if (nullptr == out_labels) {
    AERROR << "Input null out_labels ptr.";
    return false;
  }
    if(!get_objects_->Process(DataFrame* data_frame, float* points_array, int num_points)){
    return false;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
