#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <sensor.hpp>
#include <devicemanager.hpp>
#include <device.hpp>
#include <FileValue.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>
#include <cmath>
#include <iostream>
#include <chrono>         // std::chrono::seconds
#include <thread>
struct feedback{
  glm::mat4 cyclops_mat;
  glm::mat4 screen_mat;
  glm::mat4 model_mat;
  unsigned recon_mode;
};

int main(int argc, char* argv[]){

  unsigned head_tracking_id = 0;
  unsigned model_tracking_id = 0;
  CMDParser p("socket");
  p.addOpt("g",1,"glasses_tracking_id", "default: 0");
  p.addOpt("m",1,"model_tracking_id", "default: 0");
  p.addOpt("r",1,"reconmodefilename", "default: none");

  p.init(argc,argv);

  if(p.isOptSet("g")){
    head_tracking_id = p.getOptsInt("g")[0];
  }

  if(p.isOptSet("m")){
    model_tracking_id = p.getOptsInt("m")[0];
  }

  sensor::FileValue* recon_mode = 0;
  if(p.isOptSet("r")){
    recon_mode = new sensor::FileValue(p.getOptsString("r")[0].c_str());
  }
  std::string socket_name(p.getArgs()[0]);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));

  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());

  // prepare tracking system
  sensor::device* d = sensor::devicemanager::the()->get_dtrack(5000, sensor::timevalue::const_050_ms);
  sensor::sensor* head = new sensor::sensor(d, head_tracking_id);
  sensor::sensor* model = new sensor::sensor(d, model_tracking_id);

  

  glm::mat4 scale_mat;
  scale_mat[0][0] = 0.19;
  scale_mat[1][1] = 0.19;
  scale_mat[2][2] = 0.19;

  glm::mat4 trans_mat;
  trans_mat[3][0] = 0.0;
  trans_mat[3][1] = 0.15;
  trans_mat[3][2] = 0.0;
  
  feedback fb;
  fb.recon_mode = 1;
  fb.cyclops_mat[3][0] = 0.0;
  fb.cyclops_mat[3][1] = 1.8;
  fb.cyclops_mat[3][2] = 0.0;

  fb.screen_mat[3][0] = 0.0;
  fb.screen_mat[3][1] = 1.5;
  fb.screen_mat[3][2] = -1.7;

  // initial model is at center screen position
  fb.model_mat = fb.screen_mat * (trans_mat * scale_mat);
  

  while(true){

    // receive data from tracking system
    if((head_tracking_id) != 0 && (model_tracking_id != 0)){
      fb.cyclops_mat = head->getMatrix();
      fb.model_mat = model->getMatrix() * (trans_mat * scale_mat);
    }

    if(recon_mode != 0){
      recon_mode->read(fb.recon_mode);
    }

    zmq::message_t zmqm(sizeof(feedback));
    memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) &fb, sizeof(feedback));
    socket.send(zmqm);
    std::this_thread::sleep_for( std::chrono::milliseconds(16) );

  }

  return 0;
}
