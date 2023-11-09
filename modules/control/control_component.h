#pragma once

#include <memory>

#include "cyber/component/timer_component.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/local_view.pb.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::Header;
using apollo::common::Status;
using apollo::common::monitor::MonitorMessage;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using common::monitor::MonitorLogBuffer;
using std::string;

class ControlComponent final : public apollo::cyber::TimerComponent {
 public:
  ControlComponent();
  bool Init() override;
  bool Proc() override;

 private:
  void OnChassis(const std::shared_ptr<Chassis> &chassis);
  void OnPlanning(const std::shared_ptr<ADCTrajectory> &trajectory);
  void OnLocalization(
      const std::shared_ptr<LocalizationEstimate> &localization);
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  Status ProduceControlCommand(ControlCommand *control_command);
  Status CheckInput(LocalView *local_view);
  Status CheckTimestamp(const LocalView &local_view);
  void ResetAndProduceZeroControlCommand(ControlCommand *control_command);

 private:
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Reader<LocalizationEstimate>> localization_reader_;
  std::shared_ptr<Reader<ADCTrajectory>> trajectory_reader_;
  std::shared_ptr<Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<Writer<ControlCommand>> control_cmd_writer_;

  Chassis latest_chassis_;
  LocalizationEstimate latest_localization_;
  ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;

  LocalView local_view_;

  ControlConf control_conf_;

  ControllerAgent controller_agent_;
  Header latest_replan_trajectory_header_;
  std::mutex mutex_;

  bool estop_ = false;
  string estop_reason_;
  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  std::shared_ptr<DependencyInjector> injector_;
  MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo