#include <udpconnection.hpp>
#include <iostream>
int main(void){


  udpconnection c;
  c.open_receiving_socket(7002);

  std::cout << "start recv" << std::endl;
  

  size_t timestamp = 0;
  unsigned voxel_count = 0;
  const unsigned max_voxels_per_packet = 6000;
  const unsigned byte_per_voxel = 10;
  const unsigned byte_of_header = 16;
  const unsigned max_bufflen = max_voxels_per_packet * byte_per_voxel + byte_of_header;
    
  unsigned char buff[max_bufflen];


  while(1){
    size_t bytes_received = c.recv(buff, max_bufflen);
    std::cout << "bytes_received: " << bytes_received << std::endl;
    //sleep(sensor::timevalue::const_999_us * 10);
  }


  return 0;
}

