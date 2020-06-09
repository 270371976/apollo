
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

namespace apollo {
namespace vtd_bridge {

class Socket {
public:
  Socket(std::string hostname = "127.0.0.1", unsigned int port = 48191, unsigned int type = 0);
  ~Socket();
  bool connect();
  ssize_t send(const char *data, const size_t size);
  ssize_t receive(char *data, const size_t size);

public:
  std::string ip_;
  unsigned int port_;
  int sock_;
  unsigned int type_;  // UDP:0,TCP:1
  struct sockaddr_in addr_;
};

}  // namespace vtd_bridge
}  // namespace apollo