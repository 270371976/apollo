#pragma once

#include "modules/common/status/status.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/proto/control_conf.pb.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

class Controller {
 public:
  Controller() = default;
  virtual ~Controller() = default;
  virtual Status Init(std::shared_ptr<DependencyInjector> injector,
                      const ControlConf *control_conf) = 0;

  virtual Status ComputeControlCommand(const LocalizationEstimate *localization,
                                       const Chassis *chassis,
                                       const ADCTrajectory *trajectory,
                                       ControlCommand *cmd) = 0;
  virtual Status Reset() = 0;

  virtual std::string Name() const = 0;

  virtual void Stop() = 0;
};

}  // namespace control
}  // namespace apollo
