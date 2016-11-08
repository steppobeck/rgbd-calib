#ifndef UDPCONNECTION_H
#define UDPCONNECTION_H

#include <arpa/inet.h>
#include <sys/socket.h>



class udpconnection{
 public:
  udpconnection();
  ~udpconnection();

  bool open_sending_socket(const char* ip, unsigned short port);
  bool open_receiving_socket(unsigned short port);

  bool send(const void* data, size_t len);
  size_t recv(void* data, size_t len, bool blocking);


 private:
  int m_socket_desc;
  struct sockaddr_in m_socket;
};

#endif // #ifndef UDPCONNECTION_H
