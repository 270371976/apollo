
#include "modules/vtd_bridge/bridge_component/utils/Socket.h"

#include "cyber/common/log.h"

namespace apollo {
namespace vtd_bridge {

Socket::Socket(std::string ip, unsigned int port, unsigned int type) : ip_(ip), port_(port), sock_(-1), type_(type) {
  if (type_ != 0) {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);  // TCP
  } else {
    sock_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);  // SOCK_DGRAM | SOCK_NONBLOCK
  }
  if (sock_ == -1) {
    AERROR << "socket failed:";
    return;
  }
  int opt = 1;
  if (type_ != 0) {
    setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
  } else {
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  }

  memset((char *)&addr_, 0, sizeof(addr_));
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(port_);
  addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
  connect();
}

Socket::~Socket() {
  close(sock_);
  sock_ = -1;
}

bool Socket::connect() {
  bool Connected = false;
  while (!Connected) {
    if (::connect(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) == -1) {
      AERROR << "Connect failed:";
    } else {
      Connected = true;
    }
  }
  return true;
}

ssize_t Socket::send(const char *data, const size_t size) {
  ssize_t retVal = sendto(sock_, data, size, 0, (struct sockaddr *)&addr_, sizeof(addr_));
  return retVal;
}

ssize_t Socket::receive(char *data, const size_t size) {
  ssize_t retVal = recvfrom(sock_, data, size, 0, nullptr, nullptr);
  return retVal;
}

}  // namespace vtd_bridge
}  // namespace apollo