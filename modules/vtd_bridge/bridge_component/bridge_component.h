#pragma once

#include <memory>
#include <vector>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

#include "modules/vtd_bridge/bridge_component/proto/bridge_component_config.pb.h"

#include "modules/common_msgs/transform_msgs/transform.pb.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/task/task.h"
#include "modules/vtd_bridge/bridge_component/utils/DecodeMsg.h"
#include "modules/vtd_bridge/bridge_component/utils/RDBHandler.h"
#include "modules/vtd_bridge/bridge_component/utils/Socket.h"

namespace apollo {
namespace vtd_bridge {

class VtdBridge final : public apollo::cyber::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;

private:
  void ProcSend();
  void ProcReceive();
  void OnLocalization(DecodeMsg &parse_msg);
  void OnChassis(DecodeMsg &parse_msg);
  void OnPrediction(DecodeMsg &parse_msg);
  void OnTrafficLight(DecodeMsg &parse_msg);
  void OnPerceptionObstacle(DecodeMsg &parse_msg);
  void OnControl(const std::shared_ptr<control::ControlCommand> &control_cmd);

private:
  uint32_t insimframe_ = 0;
  double start_time_ = 0;
  double current_time_ = 0;
  double ts_ = 0.0;
  std::shared_ptr<Socket> send_sock_ = nullptr;
  std::shared_ptr<Socket> rece_sock_ = nullptr;

  apollo::vtd_bridge::VtdBridgeConfig config_;

  std::shared_ptr<cyber::Reader<control::ControlCommand>> control_cmd_reader_ = nullptr;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>> localization_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<transform::TransformStampeds>> tf2_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<canbus::Chassis>> chassis_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<prediction::PredictionObstacles>> prediction_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<perception::TrafficLightDetection>> traffic_light_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>> perception_obstacles_talker_ = nullptr;
};

CYBER_REGISTER_COMPONENT(VtdBridge)

}  // namespace vtd_bridge
}  // namespace apollo
