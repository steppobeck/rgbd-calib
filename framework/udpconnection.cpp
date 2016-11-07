#include "udpconnection.hpp"

#include <iostream>
#include <string.h>
#include <unistd.h>

udpconnection::udpconnection()
  : m_socket_desc(0),
    m_socket()
{}


udpconnection::~udpconnection(){
  close(m_socket_desc);
}

bool
udpconnection::open_sending_socket(const char* ip, unsigned short port){
  if ( (m_socket_desc=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
    std::cerr << "ERROR: unable to create socket" << std::endl;
    return false;
  }


  memset((char *) &m_socket, 0, sizeof(m_socket));
  m_socket.sin_family = AF_INET;
  m_socket.sin_port = htons(port);

  if (inet_aton(ip , &m_socket.sin_addr) == 0){
    std::cerr << "ERROR: unable to connect to " << ip << std::endl;
    return false;
  }

  return true;
}


bool
udpconnection::open_receiving_socket(unsigned short port){
    
  if ((m_socket_desc=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
    std::cerr << "ERROR: unable to create socket" << std::endl;
    return false;
  }
  memset((char *) &m_socket, 0, sizeof(m_socket));
  m_socket.sin_family = AF_INET;
  m_socket.sin_port = htons(port);
  m_socket.sin_addr.s_addr = htonl(INADDR_ANY);

  //bind socket to port
  if( bind(m_socket_desc , (struct sockaddr*)&m_socket, sizeof(m_socket) ) == -1){
    std::cerr << "ERROR: unable to bind socket" << std::endl;
    return false;
  }

  return true;
}


bool
udpconnection::send(const void* data, size_t len){
  if(sendto(m_socket_desc, data, len, 0 , (struct sockaddr *) &m_socket, sizeof(m_socket)) == -1){
    std::cerr << "ERROR: unable to send data" << std::endl;
    return false;
  }
  return true;
}


size_t
udpconnection::recv(void* data, size_t len, bool blocking){
  //try to receive some data, this is a blocking call
  size_t recv_len = 0;
  socklen_t slen;
  struct sockaddr_in si_other;
  recv_len = recvfrom(m_socket_desc, data, len, blocking ? 0 : MSG_DONTWAIT, (struct sockaddr *) &si_other, &slen);
  return recv_len;
}
