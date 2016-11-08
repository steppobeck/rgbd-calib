#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>

#include <iostream>


int main(int argc, char* argv[]){


  CMDParser p("socket");
  p.init(argc,argv);

  std::string socket_name(p.getArgs()[0]);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);;
#if ZMQ_VERSION_MAJOR < 3
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
#endif 
  std::string endpoint("tcp://" + socket_name);
  socket.connect(endpoint.c_str());

  while(true){



    zmq::message_t zmqm(sizeof(glm::mat4));
    socket.recv(&zmqm);

    glm::mat4 pose;
    memcpy( glm::value_ptr(pose), (const unsigned char* ) zmqm.data(), sizeof(glm::mat4));
    
    std::cout << "received: " << pose << std::endl;

  }

  return 0;
}
