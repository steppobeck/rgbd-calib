#include "rgbdsensor.hpp"

#include <cmath>
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
  
  if(config.serverport != ""){
    m_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    uint64_t hwm = 1;
    m_socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
    std::string endpoint("tcp://" + cfg.serverport);
    //std::cout << "opening socket connection to: " << endpoint << std::endl;
    m_socket.connect(endpoint.c_str());
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

glm::vec3
RGBDSensor::get_rgb_bilinear_normalized(const glm::vec2& pos_rgb){


  // convert from float coordinates to nearest interger coordinates
  const unsigned xc = std::max( 0u, 
				std::min( config.size_rgb.x - 1u, (unsigned) std::floor(pos_rgb.x)));
  const unsigned yc = std::max( 0u,
				std::min( config.size_rgb.y - 1u, (unsigned) std::floor(pos_rgb.y)));
  
  unsigned char r = frame_rgb[(yc * config.size_rgb.x * 3) + 3 * xc];
  unsigned char g = frame_rgb[(yc * config.size_rgb.x * 3) + 3 * xc + 1];
  unsigned char b = frame_rgb[(yc * config.size_rgb.x * 3) + 3 * xc + 2];

  glm::vec3 rgb;

  rgb.x = r/255.0;
  rgb.y = g/255.0;
  rgb.z = b/255.0;
  return rgb;

  // BROKEN FROM HERE DON'T KNOW WHY

  // calculate weights and boundaries along x direction
  const unsigned xa = std::floor(pos_rgb.x);
  const unsigned xb = std::ceil(pos_rgb.x);
  const double w_xb = (pos_rgb.x - xa)/255.0;
  const double w_xa = (1.0 - w_xb)/255.0;

  // calculate weights and boundaries along y direction
  const unsigned ya = std::floor(pos_rgb.y);
  const unsigned yb = std::ceil(pos_rgb.y);
  const double w_yb = (pos_rgb.y - ya)/255.0;
  const double w_ya = (1.0 - w_yb)/255.0;


    
  // calculate indices to access data
  const unsigned idmax = 3u * config.size_rgb.x * config.size_rgb.y - 2u;
  const unsigned id00 = std::min( ya * 3u * config.size_rgb.x + 3u * xa  , idmax);
  const unsigned id10 = std::min( ya * 3u * config.size_rgb.x + 3u * xb  , idmax);
  const unsigned id01 = std::min( yb * 3u * config.size_rgb.x + 3u * xa  , idmax);
  const unsigned id11 = std::min( yb * 3u * config.size_rgb.x + 3u * xb  , idmax);
  

  // RED CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb[id00] + w_xb * frame_rgb[id10];
    const float tmp_yb = w_xa * frame_rgb[id01] + w_xb * frame_rgb[id11];
    // 2. interpolate between y direction;
    rgb.x = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  // GREEN CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb[id00 + 1] + w_xb * frame_rgb[id10 + 1];
    const float tmp_yb = w_xa * frame_rgb[id01 + 1] + w_xb * frame_rgb[id11 + 1];
    // 2. interpolate between y direction;
    rgb.y = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  // BLUE CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb[id00 + 2] + w_xb * frame_rgb[id10 + 2];
    const float tmp_yb = w_xa * frame_rgb[id01 + 2] + w_xb * frame_rgb[id11 + 2];
    // 2. interpolate between y direction;
    rgb.z = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  return rgb;

}
