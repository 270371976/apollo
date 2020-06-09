#include "modules/vtd_bridge/bridge_component/bridge_component.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"
#include "modules/localization/common/localization_gflags.h"

#define DEFAULT_BUFFER 204800

namespace apollo {
namespace vtd_bridge {

using apollo::cyber::Clock;
using apollo::cyber::ComponentBase;
using apollo::localization::LocalizationEstimate;

bool VtdBridge::Init() {
  ACHECK(ComponentBase::GetProtoConfig(&config_)) << "failed to load bridge_component config file " << ComponentBase::ConfigFilePath();

  AINFO << "Load config succedded.\n" << config_.DebugString();
  //读取配置参数
  ts_ = config_.ts();

  send_sock_.reset(new Socket(config_.remote_ip(), config_.remote_send_port(), 0));     // UDP 48191
  rece_sock_.reset(new Socket(config_.remote_ip(), config_.remote_receive_port(), 1));  // TCP 48190

  localization_talker_ = node_->CreateWriter<localization::LocalizationEstimate>(FLAGS_localization_topic);
  tf2_talker_ = node_->CreateWriter<transform::TransformStampeds>(FLAGS_tf_topic);
  chassis_talker_ = node_->CreateWriter<canbus::Chassis>(FLAGS_chassis_topic);
  prediction_talker_ = node_->CreateWriter<prediction::PredictionObstacles>(FLAGS_prediction_topic);
  traffic_light_talker_ = node_->CreateWriter<perception::TrafficLightDetection>(FLAGS_traffic_light_detection_topic);
  perception_obstacles_talker_ = node_->CreateWriter<perception::PerceptionObstacles>(FLAGS_perception_obstacle_topic);

  control_cmd_reader_ = node_->CreateReader<control::ControlCommand>(FLAGS_control_command_topic, [this](const std::shared_ptr<control::ControlCommand>& control_cmd) { OnControl(control_cmd); });
  start_time_ = Clock::NowInSeconds();
  AINFO << "Init VtdBridge succedded.";
  return true;
}

bool VtdBridge::Proc() {
  AINFO << "--------------------Start processing--------------------";
  current_time_ = Clock::NowInSeconds();
  AINFO << "TimeStamp: " << current_time_;
  ProcSend();
  ProcReceive();
  AINFO << "Cost time: " << Clock::NowInSeconds() - current_time_;
  AINFO << "--------------------_End processing_--------------------";
  return true;
}

void VtdBridge::ProcSend() {
  Framework::RDBHandler myHandler;
  myHandler.initMsg();

  RDB_TRIGGER_t* trigger = (RDB_TRIGGER_t*)myHandler.addPackage(insimframe_ * ts_, insimframe_, RDB_PKG_ID_TRIGGER);  // current_time_ - start_time_;  // insimframe_ * ts_;
  if (!trigger)
    return;
  trigger->frameNo = insimframe_;
  trigger->deltaT = ts_;
  if (rece_sock_->send((char*)(myHandler.getMsg()), myHandler.getMsgTotalSize()) < 0) {
    AERROR << "ERROR in sendto";
  }

  insimframe_++;
}

void VtdBridge::ProcReceive() {
  char* szBuffer = new char[DEFAULT_BUFFER];
  ssize_t bytesInBuffer = rece_sock_->receive(szBuffer, DEFAULT_BUFFER);
  if (bytesInBuffer == -1) {
    return;
  }
  char* pData = (char*)calloc(1, bytesInBuffer);
  memcpy(pData, szBuffer, bytesInBuffer);
  DecodeMsg parse_msg = DecodeMsg(pData, bytesInBuffer);

  OnLocalization(parse_msg);
  OnChassis(parse_msg);
  OnPrediction(parse_msg);
  OnTrafficLight(parse_msg);
  OnPerceptionObstacle(parse_msg);
}

void VtdBridge::OnLocalization(DecodeMsg& parse_msg) {
  auto pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_OBJECT_STATE);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_OBJECT_STATE_t*)pdata;
    if (ptr->base.id == 1) {
      LocalizationEstimate localization;
      common::util::FillHeader(node_->Name(), &localization);
      localization.set_measurement_time(localization.header().timestamp_sec());
      auto pose = localization.mutable_pose();
      // 坐标系原点相对于大地坐标系的位置 + 速度 + 加速度 + 角度 + 角速度 + 四元数
      auto position = pose->mutable_position();
      auto linear_velocity = pose->mutable_linear_velocity();
      auto linear_acceleration = pose->mutable_linear_acceleration();
      auto euler_angles = pose->mutable_euler_angles();
      auto angular_velocity = pose->mutable_angular_velocity();

      position->set_x(ptr->base.pos.x);
      position->set_y(ptr->base.pos.y);
      position->set_z(ptr->base.pos.z);
      linear_velocity->set_x(ptr->ext.speed.x);
      linear_velocity->set_y(ptr->ext.speed.y);
      linear_velocity->set_z(ptr->ext.speed.z);
      linear_acceleration->set_x(ptr->ext.accel.x);
      linear_acceleration->set_y(ptr->ext.accel.y);
      linear_acceleration->set_z(ptr->ext.accel.z);
      euler_angles->set_x(ptr->base.pos.r);
      euler_angles->set_y(ptr->base.pos.p);
      euler_angles->set_z(ptr->base.pos.h - M_PI / 2);
      angular_velocity->set_x(-ptr->ext.speed.p);
      angular_velocity->set_y(ptr->ext.speed.r);
      angular_velocity->set_z(ptr->ext.speed.h);

      // 相对于大地坐标系的航向 pose->set_heading(ptr->base.pos.h);
      auto euler_angles_zxyd = apollo::common::math::EulerAnglesZXYd(euler_angles->x(), euler_angles->y(), euler_angles->z());
      auto quaternion = euler_angles_zxyd.ToQuaternion();
      pose->set_heading(common::math::QuaternionToHeading(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()));

      // 四元数
      auto orientation = pose->mutable_orientation();  // RPH
      orientation->set_qw(quaternion.w());
      orientation->set_qx(quaternion.x());
      orientation->set_qy(quaternion.y());
      orientation->set_qz(quaternion.z());

      // 相对于车体坐标系的加速度 + 角速度
      auto linear_acceleration_vrf = pose->mutable_linear_acceleration_vrf();
      auto angular_velocity_vrf = pose->mutable_angular_velocity_vrf();

      // auto vec_linear_velocity = Eigen::Vector3d(linear_velocity->x(), linear_velocity->y(), linear_velocity->z());
      auto vec_linear_acceleration = Eigen::Vector3d(linear_acceleration->x(), linear_acceleration->y(), linear_acceleration->z());
      auto vec_angular_velocity = Eigen::Vector3d(angular_velocity->x(), angular_velocity->y(), angular_velocity->z());
      // auto vec_linear_velocity_vrf = quaternion.inverse() * vec_linear_velocity;
      auto vec_linear_acceleration_vrf = quaternion.inverse() * vec_linear_acceleration;
      auto vec_angular_velocity_vrf = quaternion.inverse() * vec_angular_velocity;
      linear_acceleration_vrf->set_x(vec_linear_acceleration_vrf(0));
      linear_acceleration_vrf->set_y(vec_linear_acceleration_vrf(1));
      linear_acceleration_vrf->set_z(vec_linear_acceleration_vrf(2));
      angular_velocity_vrf->set_x(vec_angular_velocity_vrf(0));
      angular_velocity_vrf->set_y(vec_angular_velocity_vrf(1));
      angular_velocity_vrf->set_z(vec_angular_velocity_vrf(2));

      localization_talker_->Write(localization);

      apollo::transform::TransformStampeds transformstampeds;
      common::util::FillHeader(node_->Name(), &transformstampeds);
      auto transformstamped = transformstampeds.add_transforms();
      auto translation = transformstamped->mutable_transform()->mutable_translation();
      auto rotation = transformstamped->mutable_transform()->mutable_rotation();
      transformstamped->mutable_header()->set_frame_id(FLAGS_broadcast_tf_frame_id);
      transformstamped->mutable_header()->set_timestamp_sec(transformstampeds.header().timestamp_sec());

      transformstamped->set_child_frame_id(FLAGS_broadcast_tf_child_frame_id);
      translation->set_x(localization.pose().position().x());
      translation->set_y(localization.pose().position().y());
      translation->set_z(localization.pose().position().z());
      rotation->set_qx(localization.pose().orientation().qx());
      rotation->set_qy(localization.pose().orientation().qy());
      rotation->set_qz(localization.pose().orientation().qz());
      rotation->set_qw(localization.pose().orientation().qw());
      tf2_talker_->Write(transformstampeds);

      break;
    }
  }
}

void VtdBridge::OnChassis(DecodeMsg& parse_msg) {
  std::vector<char*> pdatas;
  canbus::Chassis chassis;
  common::util::FillHeader(node_->Name(), &chassis);
  chassis.set_engine_started(true);
  chassis.set_driving_mode(canbus::Chassis::COMPLETE_AUTO_DRIVE);
  pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_OBJECT_STATE);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_OBJECT_STATE_t*)pdata;
    if (ptr->base.id == 1) {
      auto euler_angles_zxyd = apollo::common::math::EulerAnglesZXYd(ptr->base.pos.r, ptr->base.pos.p, ptr->base.pos.h - M_PI / 2);
      auto quaternion = euler_angles_zxyd.ToQuaternion();
      auto vec_linear_velocity = Eigen::Vector3d(ptr->ext.speed.x, ptr->ext.speed.y, ptr->ext.speed.z);
      auto vec_linear_velocity_vrf = quaternion.inverse() * vec_linear_velocity;
      chassis.set_speed_mps(vec_linear_velocity_vrf(1));
      break;
    }
  }

  pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_DRIVER_CTRL);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_DRIVER_CTRL_t*)pdata;
    if (ptr->playerId == 1) {
      chassis.set_throttle_percentage(ptr->throttlePedal * 100);
      chassis.set_brake_percentage(ptr->brakePedal * 100);
      chassis.set_steering_percentage(ptr->steeringTgt / (27.50 * M_PI / 180) * 100);
      if (ptr->gear == RDB_GEAR_BOX_POS_P) {
        chassis.set_gear_location(canbus::Chassis::GEAR_PARKING);
      } else if (ptr->gear == RDB_GEAR_BOX_POS_R) {
        chassis.set_gear_location(canbus::Chassis::GEAR_REVERSE);
      } else if (ptr->gear == RDB_GEAR_BOX_POS_N) {
        chassis.set_gear_location(canbus::Chassis::GEAR_NEUTRAL);
      } else if (ptr->gear == RDB_GEAR_BOX_POS_D) {
        chassis.set_gear_location(canbus::Chassis::GEAR_DRIVE);
      } else {
        chassis.set_gear_location(canbus::Chassis::GEAR_NONE);
      }
      break;
    }
  }
  pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_VEHICLE_SYSTEMS);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_VEHICLE_SYSTEMS_t*)pdata;
    if (ptr->playerId == 1) {
      chassis.mutable_signal()->set_high_beam(ptr->lightMask & RDB_VEHICLE_LIGHT_HIGH_BEAM);
      chassis.mutable_signal()->set_low_beam(ptr->lightMask & RDB_VEHICLE_LIGHT_LOW_BEAM);
      chassis.mutable_signal()->set_emergency_light(ptr->lightMask & RDB_VEHICLE_LIGHT_EMERGENCY);
      if (ptr->lightMask & RDB_VEHICLE_LIGHT_INDICATOR_L) {
        chassis.mutable_signal()->set_turn_signal(common::VehicleSignal::TURN_LEFT);
      } else if (ptr->lightMask & RDB_VEHICLE_LIGHT_INDICATOR_R) {
        chassis.mutable_signal()->set_turn_signal(common::VehicleSignal::TURN_RIGHT);
      } else {
        chassis.mutable_signal()->set_turn_signal(common::VehicleSignal::TURN_NONE);
      }
      break;
    }
  }
  chassis_talker_->Write(chassis);
}

void VtdBridge::OnPrediction(DecodeMsg& parse_msg) {
  prediction::PredictionObstacles prediction_obstacles;
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  // prediction_talker_->Write(prediction_obstacles);
}

void VtdBridge::OnTrafficLight(DecodeMsg& parse_msg) {
  perception::TrafficLightDetection traffic_light_detection;
  common::util::FillHeader(node_->Name(), &traffic_light_detection);
  auto pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_TRAFFIC_LIGHT);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_TRAFFIC_LIGHT_t*)pdata;
    auto traffic_light = traffic_light_detection.add_traffic_light();
    traffic_light->set_id(std::to_string(ptr->base.id));
    if (ptr->base.stateMask == 0x10000000) {
      traffic_light->set_color(perception::TrafficLight::RED);
    } else if (ptr->base.stateMask == 0x01000000) {
      traffic_light->set_color(perception::TrafficLight::YELLOW);
    } else if (ptr->base.stateMask == 0x00100000) {
      traffic_light->set_color(perception::TrafficLight::GREEN);
    } else {
      traffic_light->set_color(perception::TrafficLight::UNKNOWN);
    }
    traffic_light->set_confidence(1.0);
  }
  traffic_light_detection.set_contain_lights(traffic_light_detection.traffic_light_size() > 0);
  traffic_light_talker_->Write(traffic_light_detection);
}

void VtdBridge::OnPerceptionObstacle(DecodeMsg& parse_msg) {
  perception::PerceptionObstacles perception_obstacles;
  common::util::FillHeader(node_->Name(), &perception_obstacles);
  auto pdatas = parse_msg.GetPkgByPkgId(RDB_PKG_ID_OBJECT_STATE);
  for (auto pdata : pdatas) {
    auto ptr = (RDB_OBJECT_STATE_t*)pdata;
    if (ptr->base.id != 1) {
      auto perception_obstacle = perception_obstacles.add_perception_obstacle();
      perception_obstacle->set_timestamp(perception_obstacles.header().timestamp_sec());
      perception_obstacle->set_id(ptr->base.id);

      // 参考坐标系原点全局位姿
      auto vrf_position = Eigen::Vector3d(ptr->base.pos.x, ptr->base.pos.y, ptr->base.pos.z);
      // 相对于大地坐标系的航向 pose->set_heading(ptr->base.pos.h);
      auto euler_angles_zxyd = apollo::common::math::EulerAnglesZXYd(ptr->base.pos.r, ptr->base.pos.p, ptr->base.pos.h - M_PI / 2);
      auto quaternion = euler_angles_zxyd.ToQuaternion();
      perception_obstacle->set_theta(common::math::QuaternionToHeading(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()));
      // 几何中心在参考坐标系下的坐标
      auto center_position_vrf = Eigen::Vector3d(-ptr->base.geo.offY, ptr->base.geo.offX, ptr->base.geo.offZ);
      // 几何中心在全局坐标系下的位姿
      auto center_position = quaternion * center_position_vrf + vrf_position;

      auto position = perception_obstacle->mutable_position();
      auto velocity = perception_obstacle->mutable_velocity();
      auto acceleration = perception_obstacle->mutable_acceleration();

      position->set_x(center_position(0));
      position->set_y(center_position(1));
      position->set_z(center_position(2));
      velocity->set_x(ptr->ext.speed.x);
      velocity->set_y(ptr->ext.speed.y);
      velocity->set_z(ptr->ext.speed.z);
      acceleration->set_x(ptr->ext.accel.x);
      acceleration->set_y(ptr->ext.accel.y);
      acceleration->set_z(ptr->ext.accel.z);

      perception_obstacle->set_length(ptr->base.geo.dimX);
      perception_obstacle->set_width(ptr->base.geo.dimY);
      perception_obstacle->set_height(ptr->base.geo.dimZ);
      if (ptr->base.type == RDB_OBJECT_TYPE_PLAYER_CAR || ptr->base.type == RDB_OBJECT_TYPE_PLAYER_TRUCK || ptr->base.type == RDB_OBJECT_TYPE_PLAYER_BUS) {
        perception_obstacle->set_type(perception::PerceptionObstacle::VEHICLE);
      } else if (ptr->base.type == RDB_OBJECT_TYPE_PLAYER_BIKE || ptr->base.type == RDB_OBJECT_TYPE_PLAYER_MOTORBIKE) {
        perception_obstacle->set_type(perception::PerceptionObstacle::BICYCLE);
      } else if (ptr->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN) {
        perception_obstacle->set_type(perception::PerceptionObstacle::PEDESTRIAN);
      } else {
        perception_obstacle->set_type(perception::PerceptionObstacle::UNKNOWN);
      }
    }
  }
  perception_obstacles_talker_->Write(perception_obstacles);
}

void VtdBridge::OnControl(const std::shared_ptr<control::ControlCommand>& control_cmd) {
  Framework::RDBHandler myHandler;
  myHandler.initMsg();

  RDB_DRIVER_CTRL_t* control_cmd_VTD = (RDB_DRIVER_CTRL_t*)myHandler.addPackage(insimframe_ * ts_, insimframe_, RDB_PKG_ID_DRIVER_CTRL);
  if (!control_cmd_VTD)
    return;

  control_cmd_VTD->playerId = 1;
  control_cmd_VTD->throttlePedal = control_cmd->throttle() / 100;
  control_cmd_VTD->brakePedal = control_cmd->brake() / 100;
  control_cmd_VTD->steeringWheel = control_cmd->steering_target() / 100 * 10;
  if (control_cmd->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    control_cmd_VTD->gear = RDB_GEAR_BOX_POS_N;
  } else if (control_cmd->gear_location() == canbus::Chassis::GEAR_DRIVE) {
    control_cmd_VTD->gear = RDB_GEAR_BOX_POS_D;
  } else if (control_cmd->gear_location() == canbus::Chassis::GEAR_REVERSE) {
    control_cmd_VTD->gear = RDB_GEAR_BOX_POS_R;
  } else if (control_cmd->gear_location() == canbus::Chassis::GEAR_PARKING) {
    control_cmd_VTD->gear = RDB_GEAR_BOX_POS_P;
  }
  if (control_cmd->turnsignal() == control::TURN_LEFT) {
    control_cmd_VTD->flags = RDB_DRIVER_FLAG_INDICATOR_L;
  } else if (control_cmd->turnsignal() == control::TURN_RIGHT) {
    control_cmd_VTD->flags = RDB_DRIVER_FLAG_INDICATOR_R;
  } else {
    control_cmd_VTD->flags = RDB_DRIVER_FLAG_NONE;
  }
  control_cmd_VTD->validityFlags
    = RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL | RDB_DRIVER_INPUT_VALIDITY_THROTTLE | RDB_DRIVER_INPUT_VALIDITY_BRAKE | RDB_DRIVER_INPUT_VALIDITY_GEAR | RDB_DRIVER_INPUT_VALIDITY_FLAGS;

  if (rece_sock_->send((char*)(myHandler.getMsg()), myHandler.getMsgTotalSize()) < 0) {
    AERROR << "ERROR in sendto";
  }
}

}  // namespace vtd_bridge
}  // namespace apollo
