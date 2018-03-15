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
  unsigned stream_slot;
};

int main(int argc, char* argv[]){


  unsigned model_tracking_id = 0;
  unsigned art_port = 5000;
  unsigned start_port = 9000;
  std::string sender_ip = "127.0.0.1";

  CMDParser p("glass_ids");
  p.addOpt("m",1,"model_tracking_id", "default: 0");
  p.addOpt("r",1,"reconmodefilename", "specifiy a file for selecting the reconstruction mode default: none");
  p.addOpt("x",1,"streamslotfilename", "specifiy a file for selecting the stream slot default: none");
  p.addOpt("a",1,"artport", "default: 5000");
  p.addOpt("s",1,"senderip", "default: 127.0.0.1");
  p.addOpt("p",1,"port", "default: 9000");


  p.init(argc,argv);

  if(p.isOptSet("m")){
    model_tracking_id = p.getOptsInt("m")[0];
  }

  if(p.isOptSet("a")){
    art_port = p.getOptsInt("a")[0];
  }

  if(p.isOptSet("p")){
    start_port = p.getOptsInt("p")[0];
  }

  if(p.isOptSet("s")){
    sender_ip = p.getOptsString("s")[0];
  }



  sensor::FileValue* recon_mode = 0;
  if(p.isOptSet("r")){
    recon_mode = new sensor::FileValue(p.getOptsString("r")[0].c_str());
  }

  sensor::FileValue* stream_slot = 0;
  if(p.isOptSet("x")){
    stream_slot = new sensor::FileValue(p.getOptsString("x")[0].c_str());
  }

  // prepare tracking system
  sensor::device* d = sensor::devicemanager::the()->get_dtrack(art_port, sensor::timevalue::const_050_ms);
  sensor::sensor* model = new sensor::sensor(d, model_tracking_id);

  zmq::context_t ctx(1); // means single threaded
  uint32_t hwm = 1;
  std::vector<zmq::socket_t*> sockets;
  std::vector<sensor::sensor*> glasses;
  // parse glass_ids and generate socket endpoints
  for(const auto& g_id : p.getArgs()){
     std::string endpoint = std::string("tcp://" + sender_ip + ":" + std::to_string(start_port));
     ++start_port;
     zmq::socket_t* socket = new zmq::socket_t(ctx, ZMQ_PUB); // means a publisher
     socket->setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
     socket->bind(endpoint.c_str());
     sockets.push_back(socket);
     glasses.push_back(new sensor::sensor(d, std::stoi(g_id)));
     std::cout << "sending glass id " << std::stoi(g_id) << " to endpoint " << endpoint <<  std::endl;
  }


  glm::mat4 scale_mat;
  scale_mat[0][0] = 0.25;
  scale_mat[1][1] = 0.25;
  scale_mat[2][2] = 0.25;

  glm::mat4 trans_mat;
  trans_mat[3][0] = 0.0;
  trans_mat[3][1] = 0.15;
  trans_mat[3][2] = 0.0;
  
  feedback fb;
  fb.recon_mode = 1;
  fb.stream_slot = 0;
  fb.cyclops_mat[3][0] = 0.0;
  fb.cyclops_mat[3][1] = 1.8;
  fb.cyclops_mat[3][2] = 0.0;

  fb.screen_mat[3][0] = 0.0; // -0.05
  fb.screen_mat[3][1] = 1.445; // 1.24
  fb.screen_mat[3][2] = -2.03;

  // initial model is at center screen position
  fb.model_mat = fb.screen_mat * (trans_mat * scale_mat);
  

  while(true){



    if(recon_mode != 0){
      recon_mode->read(fb.recon_mode);
    }

    if(stream_slot != 0){
      stream_slot->read(fb.stream_slot);
    }

    fb.model_mat = model->getMatrix() * (trans_mat * scale_mat);
    for(unsigned i = 0; i != glasses.size(); ++i){
       fb.cyclops_mat = glasses[i]->getMatrix();
       zmq::message_t zmqm(sizeof(feedback));
       memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) &fb, sizeof(feedback));
       sockets[i]->send(zmqm);
    }



    std::this_thread::sleep_for( std::chrono::milliseconds(16) );

  }


  if(recon_mode != 0){
    delete recon_mode;
  }

  if(stream_slot != 0){
    delete stream_slot;
  }


  return 0;
}

