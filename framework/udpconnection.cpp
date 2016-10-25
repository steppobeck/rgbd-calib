#include "udpconnection.hpp"
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <strings.h>

udpconnection::udpconnection(const char* hostname, unsigned short portnum):
  _hostname(hostname),
  _portnum(portnum),
  _sock(0),
  _me(),
  _other(),
  _blocking(false)
{}


udpconnection::~udpconnection(){

}


bool
udpconnection::open(){

  _me.sin_addr.s_addr = 0;
  _me.sin_family = AF_INET;
  _me.sin_port = htons(_portnum);

  struct hostent *he;
  if ((he=gethostbyname(_hostname.c_str())) == 0){
      std::cerr << "Error by Client: hostname "
                << _hostname
                << " unknown"
                << std::endl;
      return false;
  }


  _other.sin_family = AF_INET;
  _other.sin_port = htons(_portnum);
  _other.sin_addr = *((struct in_addr *)he->h_addr);
  bzero(&(_other.sin_zero), 8);


  /* Socket anlegen und binden */
  if ((_sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
    std::cerr << "udpconnection::open(): cannot create socket "
	      << std::endl;
    return false;
  }




  return true;
}

bool
udpconnection::bind(){
  if (::bind(_sock, (struct sockaddr *)&_me, sizeof(struct sockaddr)) == -1){
    std::cerr << "udpconnection::open(): cannot bind socket "
	      << std::endl;
    return false;
  }
  return true;
}

bool
udpconnection::send(const void* data, size_t len){
  socklen_t l = sizeof (_other);
  if(!_blocking)
    sendto( _sock, data, len, MSG_DONTWAIT, (struct sockaddr* )&_other, l);
  else
    sendto( _sock, data, len, 0, (struct sockaddr* )&_other, l);
  return true;
}

int
udpconnection::recv(void* data, size_t len){

  socklen_t l = sizeof (_other);
  int bytes_received;
  if(!_blocking)
    bytes_received = recvfrom( _sock, data, len, MSG_DONTWAIT, (struct sockaddr* )&_other, &l);
  else
    bytes_received = recvfrom( _sock, data, len, 0, (struct sockaddr* )&_other, &l);

  return bytes_received;
}



void
udpconnection::setBlocking(bool on){
  _blocking = on;
}
