#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/vtd_bridge/bridge_component/utils/RDBHandler.h"
namespace apollo {
namespace vtd_bridge {

class DecodeMsg {
public:
  DecodeMsg(void* pdata, unsigned int data_size);
  ~DecodeMsg();
  std::vector<char*> GetPkgByPkgId(unsigned int Id);

private:
  void parseRDBMessage(RDB_MSG_t* msg);
  void parseRDBMessageEntry(const double& simTime, const unsigned int& simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr);

private:
  char* pdata_ = nullptr;
  ssize_t data_size_ = 0;
  std::unordered_map<unsigned int, std::vector<char*>> hash_map_;
};

}  // namespace vtd_bridge
}  // namespace apollo