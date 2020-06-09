#include "modules/vtd_bridge/bridge_component/utils/DecodeMsg.h"

#include "cyber/common/log.h"

namespace apollo {
namespace vtd_bridge {

DecodeMsg::DecodeMsg(void* pdata, unsigned int data_size) : pdata_((char*)pdata), data_size_(data_size) {
  RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)pdata_;
  if (hdr->magicNo != RDB_MAGIC_NO) {
    AERROR << "message receiving is out of sync; discarding data";
    data_size_ = 0;
    return;
  }

  while (data_size >= (hdr->headerSize + hdr->dataSize)) {
    unsigned int msgSize = hdr->headerSize + hdr->dataSize;
    parseRDBMessage((RDB_MSG_t*)pdata_);

    memmove(pdata_, pdata_ + msgSize, data_size - msgSize);
    data_size -= msgSize;
  }
}
DecodeMsg::~DecodeMsg() {
  hash_map_.clear();
}

std::vector<char*> DecodeMsg::GetPkgByPkgId(unsigned int Id) {
  return hash_map_[Id];
}

void DecodeMsg::parseRDBMessage(RDB_MSG_t* msg) {
  if (!msg)
    return;

  if (!msg->hdr.dataSize)
    return;

  RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)(((char*)msg) + msg->hdr.headerSize);
  uint32_t remainingBytes = msg->hdr.dataSize;

  while (remainingBytes) {
    parseRDBMessageEntry(msg->hdr.simTime, msg->hdr.frameNo, entry);

    remainingBytes -= (entry->headerSize + entry->dataSize);

    if (remainingBytes)
      entry = (RDB_MSG_ENTRY_HDR_t*)((((char*)entry) + entry->headerSize + entry->dataSize));
  }
}

void DecodeMsg::parseRDBMessageEntry(const double& simTime, const unsigned int& simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr) {
  if (!entryHdr)
    return;

  int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

  if (!noElements) {
    switch (entryHdr->pkgId) {
    case RDB_PKG_ID_START_OF_FRAME:
      AINFO << "void parseRDBMessageEntry: got start of frame";
      break;
    case RDB_PKG_ID_END_OF_FRAME:
      AINFO << "void parseRDBMessageEntry: got end of frame";
      break;
    default:
      break;
    }
    return;
  }

  unsigned char ident = 6;
  char* dataPtr = (char*)entryHdr;

  dataPtr += entryHdr->headerSize;

  while (noElements--) {
    if (entryHdr->pkgId <= 48) {
      hash_map_[entryHdr->pkgId].push_back(dataPtr);
    }
    // switch (entryHdr->pkgId) {
    // case RDB_PKG_ID_COORD_SYSTEM:
    //   // Framework::RDBHandler::print(*((RDB_COORD_SYSTEM_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_COORD:
    //   // Framework::RDBHandler::print(*((RDB_COORD_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_ROAD_POS:
    //   // // Framework::RDBHandler::print(*((RDB_ROAD_POS_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_LANE_INFO:
    //   // Framework::RDBHandler::print(*((RDB_LANE_INFO_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_ROADMARK:
    //   // Framework::RDBHandler::print(*((RDB_ROADMARK_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_OBJECT_CFG:
    //   // Framework::RDBHandler::print(*((RDB_OBJECT_CFG_t*)dataPtr), ident);
    //   break;
    // case RDB_PKG_ID_OBJECT_STATE:
    //   // Framework::RDBHandler::print(*((RDB_OBJECT_CFG_t*)dataPtr));
    //   break;

    // case RDB_PKG_ID_VEHICLE_SYSTEMS:
    //   // Framework::RDBHandler::print(*((RDB_VEHICLE_SYSTEMS_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_VEHICLE_SETUP:
    //   // Framework::RDBHandler::print(*((RDB_VEHICLE_SETUP_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_ENGINE:
    //   // Framework::RDBHandler::print(*((RDB_ENGINE_t*)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident);
    //   break;

    // case RDB_PKG_ID_DRIVETRAIN:
    //   // Framework::RDBHandler::print(*((RDB_DRIVETRAIN_t*)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident);
    //   break;

    // case RDB_PKG_ID_WHEEL:
    //   // Framework::RDBHandler::print(*((RDB_WHEEL_t*)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident);
    //   break;

    // case RDB_PKG_ID_PED_ANIMATION:
    //   // Framework::RDBHandler::print(*((RDB_PED_ANIMATION_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_SENSOR_STATE:
    //   // Framework::RDBHandler::print(*((RDB_SENSOR_STATE_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_SENSOR_OBJECT:
    //   // Framework::RDBHandler::print(*((RDB_SENSOR_OBJECT_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_CAMERA:
    //   // Framework::RDBHandler::print(*((RDB_CAMERA_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_CONTACT_POINT:
    //   // Framework::RDBHandler::print(*((RDB_CONTACT_POINT_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_TRAFFIC_SIGN:
    //   // Framework::RDBHandler::print(*((RDB_TRAFFIC_SIGN_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_ROAD_STATE:
    //   // Framework::RDBHandler::print(*((RDB_ROAD_STATE_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_IMAGE:
    //   // Framework::RDBHandler::print(*((RDB_IMAGE_t*)dataPtr), ident);
    //   break;
    // case RDB_PKG_ID_LIGHT_MAP:
    //   // Framework::RDBHandler::print(*((RDB_IMAGE_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_LIGHT_SOURCE:
    //   // Framework::RDBHandler::print(*((RDB_LIGHT_SOURCE_t*)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident);
    //   break;

    // case RDB_PKG_ID_ENVIRONMENT:
    //   // Framework::RDBHandler::print(*((RDB_ENVIRONMENT_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_TRIGGER:
    //   // Framework::RDBHandler::print(*((RDB_TRIGGER_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_DRIVER_CTRL:
    //   // Framework::RDBHandler::print(*((RDB_DRIVER_CTRL_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_TRAFFIC_LIGHT:
    //   // Framework::RDBHandler::print(*((RDB_TRAFFIC_LIGHT_t*)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident);
    //   break;

    // case RDB_PKG_ID_DRIVER_PERCEPTION:
    //   // Framework::RDBHandler::print(*((RDB_DRIVER_PERCEPTION_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_TONE_MAPPING:
    //   // Framework::RDBHandler::print(*((RDB_FUNCTION_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_ROAD_QUERY:
    //   // Framework::RDBHandler::print(*((RDB_ROAD_QUERY_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_TRAJECTORY:
    //   // Framework::RDBHandler::print(*((RDB_TRAJECTORY_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_DYN_2_STEER:
    //   // Framework::RDBHandler::print(*((RDB_DYN_2_STEER_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_STEER_2_DYN:
    //   // Framework::RDBHandler::print(*((RDB_STEER_2_DYN_t*)dataPtr), ident);
    //   break;

    // case RDB_PKG_ID_PROXY:
    //   // Framework::RDBHandler::print(*((RDB_PROXY_t*)dataPtr), ident);
    //   break;

    // default:
    //   break;
    // }
    dataPtr += entryHdr->elementSize;
  }
}
}  // namespace vtd_bridge
}  // namespace apollo