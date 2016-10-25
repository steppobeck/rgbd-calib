#ifndef UDPCONNECTION_H
#define UDPCONNECTION_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cstdio>
#include <string>



class udpconnection{
 public:
  udpconnection(const char* hostname, unsigned short portnum);
  ~udpconnection();

  bool open();
  bool bind();
  bool send(const void* data, size_t len);
  int recv(void* data, size_t len);

  void setBlocking(bool on);
 private:
  std::string _hostname;
  unsigned short _portnum;
  int _sock;
  struct sockaddr_in _me;
  struct sockaddr_in _other;
  bool _blocking;
};


#endif // #ifndef UDPCONNECTION_H
