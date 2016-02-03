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
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());


  unsigned tick = 0;
  while(true){

    glm::mat4 pose;

    pose[3][0] = 0.0; // translate in x
    pose[3][1] = 0.1 * tick; // translate in y
    pose[3][2] = 0.0; // translate in z

    zmq::message_t zmqm(sizeof(glm::mat4));

    memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) glm::value_ptr(pose), sizeof(glm::mat4));
    socket.send(zmqm);

    std::cout << "sending: " << pose << std::endl;

    ++tick;
  }

  return 0;
}
