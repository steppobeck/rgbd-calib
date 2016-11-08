#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>

#include <iostream>

int main(int argc, char* argv[]){

  glm::vec3 position(0.0,0.0,0.0);
  CMDParser p("socket");
  p.addOpt("p",3,"position", "specify the 3D position which should be send, default 0.0 0.0 0.0");
  p.init(argc,argv);

  if(p.isOptSet("p")){
    position = glm::vec3(p.getOptsFloat("p")[0], p.getOptsFloat("p")[1], p.getOptsFloat("p")[2]);
    std::cout << "setting position to " << position << std::endl;
  }

  std::string socket_name(p.getArgs()[0]);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher
#if ZMQ_VERSION_MAJOR < 3
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
#endif 
  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());


  unsigned tick = 0;
  while(true){

    glm::mat4 pose;

    pose[3][0] = position.x;
    pose[3][1] = position.y;
    pose[3][2] = position.z;

    zmq::message_t zmqm(sizeof(glm::mat4));

    memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) glm::value_ptr(pose), sizeof(glm::mat4));
    socket.send(zmqm);

    //std::cout << "sending: " << pose << std::endl;

    ++tick;
  }

  return 0;
}
