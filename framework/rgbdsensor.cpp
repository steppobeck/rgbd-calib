#include "rgbdsensor.hpp"

#include <iostream>

RGBDSensor::RGBDSensor(const RGBDConfig& cfg)
  : config(cfg),
    frame_rgb(0),
    frame_ir(0),
    frame_d(0),
    m_ctx(1),
    m_socket(m_ctx, ZMQ_SUB)
{

  frame_rgb = new unsigned char [3 * config.size_rgb.x * config.size_rgb.y];
  frame_ir  = new unsigned char [config.size_d.x * config.size_d.y];
  frame_d   = new float         [config.size_d.x * config.size_d.y];
  

  m_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  uint64_t hwm = 1;
  m_socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + cfg.serverport);
  //std::cout << "opening socket connection to: " << endpoint << std::endl;
  m_socket.connect(endpoint.c_str());



  // init buffers with some test values here
  for(unsigned y = 0; y != config.size_rgb.y; ++y){
    for(unsigned x = 0; x != config.size_rgb.x; ++x){
      const unsigned i = y * 3 * config.size_rgb.x + 3 * x;
      frame_rgb[i] = 255;
    }
  }

  for(unsigned i = 0; i != (config.size_d.x * config.size_d.y); ++i){
    frame_d[i] = 1.0;
  }
  
}



RGBDSensor::~RGBDSensor(){
  delete [] frame_rgb;
  delete [] frame_ir;
  delete [] frame_d;
}


glm::vec3
RGBDSensor::calc_pos_d(float x, float y, float d){
  const float x_d  = ((x - config.principal_d.x)/config.focal_d.x) * d;
  const float y_d  = ((y - config.principal_d.y)/config.focal_d.y) * d;
  return glm::vec3(x_d, y_d, d);
}


glm::vec2
RGBDSensor::calc_pos_rgb(const glm::vec3& pos_d){

  glm::vec4 pos_d_H(pos_d.x, pos_d.y, pos_d.z, 1.0f);
  glm::vec4 pos_rgb_H = config.eye_d_to_eye_rgb * pos_d_H;		
  float xcf = (pos_rgb_H[0]/pos_rgb_H[2]) * config.focal_rgb.x + config.principal_rgb.x;
  float ycf = (pos_rgb_H[1]/pos_rgb_H[2]) * config.focal_rgb.y + config.principal_rgb.y;

  return glm::vec2(xcf,ycf);

}

void
RGBDSensor::recv(bool recvir){

  const unsigned bytes_rgb(3 * config.size_rgb.x * config.size_rgb.y);
  const unsigned bytes_ir(config.size_d.x * config.size_d.y);
  const unsigned bytes_d(config.size_d.x * config.size_d.y * sizeof(float));
  const unsigned bytes_recv(recvir ? bytes_rgb + bytes_ir + bytes_d :
			    bytes_rgb + bytes_d);
  zmq::message_t zmqm(bytes_recv);
  m_socket.recv(&zmqm); // blocking
  unsigned offset = 0;
  memcpy((unsigned char*) frame_rgb, (unsigned char*) zmqm.data() + offset, bytes_rgb);
  offset += bytes_rgb;
  memcpy((unsigned char*) frame_d, (unsigned char*) zmqm.data() + offset, bytes_d);
  offset += bytes_d;
  if(recvir){
    memcpy((unsigned char*) frame_ir, (unsigned char*) zmqm.data() + offset, bytes_ir);
  }

}

