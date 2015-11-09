#include "rgbdsensor.hpp"

#include <DataTypes.hpp>
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

  for(unsigned y = 0; y < config.size_rgb.y; ++y){
    for(unsigned x = 0; x < config.size_rgb.x; ++x){
      const unsigned i = y * 3 * config.size_rgb.x + 3 * x;
      const int v = (((y & 0x8) == 0) ^ ((x & 0x8) == 0)) * 255;
      frame_rgb[i] = (unsigned char) v;
      frame_rgb[i + 1] = (unsigned char) v;
      frame_rgb[i + 2] = (unsigned char) v;

    }
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
#if 1
  xcf = std::max(0.0f, std::min(xcf, config.size_rgb.x - 1.0f));
  ycf = std::max(0.0f, std::min(ycf, config.size_rgb.y - 1.0f));
#endif
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

  glm::vec3 rgb;

#if 0
  rgb.z = 0.0f;
  rgb.x = pos_rgb.x / config.size_rgb.x;
  rgb.y = pos_rgb.y / config.size_rgb.y;

  if(pos_rgb.x > config.size_rgb.x || pos_rgb.y > config.size_rgb.y){
    rgb.z = 1.0;
    return rgb;
  }
#endif


#if 1
  // calculate weights and boundaries along x direction
  const unsigned xa = std::floor(pos_rgb.x);
  const unsigned xb = std::ceil(pos_rgb.x);
  const float w_xb = (pos_rgb.x - xa);
  const float w_xa = (1.0 - w_xb);

  // calculate weights and boundaries along y direction
  const unsigned ya = std::floor(pos_rgb.y);
  const unsigned yb = std::ceil(pos_rgb.y);
  const float w_yb = (pos_rgb.y - ya);
  const float w_ya = (1.0 - w_yb);

  //std::cout << pos_rgb.x << " " << xa << " " << xb << std::endl;
  //std::cout << pos_rgb.y << " " << ya << " " << yb << std::endl;

  //rgb.x = xa * 1.0f / config.size_rgb.x;
  //rgb.y = ya * 1.0f / config.size_rgb.y;

  //rgb.x = frame_rgb[ya * 3 * config.size_rgb.x + 3 * xa];
  //rgb.x /= 255.0;
  //return rgb;


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

  rgb.x /= 255.0;
  rgb.y /= 255.0;
  rgb.z /= 255.0;

  return rgb;
#endif
}



glm::mat4
RGBDSensor::guess_eye_d_to_world(const ChessboardSampling& cbs, const Checkerboard& cb){


  // find slowest ChessboardPose
  const double time         = cbs.searchSlowestTime(cbs.searchStartIR());
  bool valid_pose;
  glm::mat4 chessboard_pose = cb.pose_offset * cbs.interpolatePose(time,valid_pose);
  if(!valid_pose){
    std::cerr << "ERROR: could not interpolate valid pose" << std::endl;
    exit(0);
  }


  // find pose of that board in kinect camera space
  bool valid_ir;
  ChessboardViewIR cbvir(cbs.interpolateIR(time, valid_ir));
  if(!valid_ir){
    std::cerr << "ERROR: could not interpolate valid ChessboardIR" << std::endl;
    exit(0);
  }


  std::vector<glm::vec3> exs;
  std::vector<glm::vec3> eys;

  const float u = cbvir.corners[0].x;
  const float v = cbvir.corners[0].y;
  const float d = cbvir.corners[0].z;
  std::cerr << "calc origin at " << u << ", " << v << ", " << d << std::endl;
  glm::vec3 origin = calc_pos_d(u,v,d);

  for(unsigned i = 1; i < (CB_WIDTH * CB_HEIGHT); ++i){

    const float u = cbvir.corners[i].x;
    const float v = cbvir.corners[i].y;
    const float d = cbvir.corners[i].z;

    glm::vec3 corner = calc_pos_d(u,v,d);
    
    if(i < CB_WIDTH){
      eys.push_back(glm::normalize(corner - origin));
    }
    else if(i % CB_WIDTH == 0){
      exs.push_back(glm::normalize(corner - origin));
    }
  }

  glm::vec3 ex(calcMean(exs));
  glm::vec3 ey(calcMean(eys));
  
  ex = glm::normalize(ex);
  ey = glm::normalize(ey);
  
  glm::vec3 ez = glm::cross(ex,ey);
  ey           = glm::cross(ez,ex);

  std::cerr << "origin: " << origin << std::endl;
  std::cerr << "ex: " << ex << std::endl;
  std::cerr << "ey: " << ey << std::endl;
  std::cerr << "ez: " << ez << std::endl;

  glm::mat4 eye_d_to_world;
  eye_d_to_world[0][0] = ex[0];
  eye_d_to_world[0][1] = ex[1];
  eye_d_to_world[0][2] = ex[2];

  eye_d_to_world[1][0] = ey[0];
  eye_d_to_world[1][1] = ey[1];
  eye_d_to_world[1][2] = ey[2];

  eye_d_to_world[2][0] = ez[0];
  eye_d_to_world[2][1] = ez[1];
  eye_d_to_world[2][2] = ez[2];

  eye_d_to_world[3][0] = origin[0];
  eye_d_to_world[3][1] = origin[1];
  eye_d_to_world[3][2] = origin[2];

  eye_d_to_world = glm::inverse(eye_d_to_world);
  
  return chessboard_pose * eye_d_to_world;

}



glm::mat4
RGBDSensor::guess_eye_d_to_world_static(const ChessboardSampling& cbs, const Checkerboard& cb){

  if(cbs.getIRs().empty() || cbs.getPoses().empty()){
    std::cerr << "ERROR in RGBDSensor::guess_eye_d_to_world_static: no Chessboard found" << std::endl;
    exit(0);
  }

  glm::mat4 chessboard_pose = cb.pose_offset * cbs.getPoses()[0].mat;
  ChessboardViewIR cbvir(cbs.getIRs()[0]);

  std::vector<glm::vec3> exs;
  std::vector<glm::vec3> eys;

  const float u = cbvir.corners[0].x;
  const float v = cbvir.corners[0].y;
  const float d = cbvir.corners[0].z;
  std::cerr << "calc origin at " << u << ", " << v << ", " << d << std::endl;
  glm::vec3 origin = calc_pos_d(u,v,d);

  for(unsigned i = 1; i < (CB_WIDTH * CB_HEIGHT); ++i){

    const float u = cbvir.corners[i].x;
    const float v = cbvir.corners[i].y;
    const float d = cbvir.corners[i].z;

    glm::vec3 corner = calc_pos_d(u,v,d);
    
    if(i < CB_WIDTH){
      eys.push_back(glm::normalize(corner - origin));
    }
    else if(i % CB_WIDTH == 0){
      exs.push_back(glm::normalize(corner - origin));
    }
  }

  glm::vec3 ex(calcMean(exs));
  glm::vec3 ey(calcMean(eys));
  
  ex = glm::normalize(ex);
  ey = glm::normalize(ey);
  
  glm::vec3 ez = glm::cross(ex,ey);
  ey           = glm::cross(ez,ex);

  std::cerr << "origin: " << origin << std::endl;
  std::cerr << "ex: " << ex << std::endl;
  std::cerr << "ey: " << ey << std::endl;
  std::cerr << "ez: " << ez << std::endl;

  glm::mat4 eye_d_to_world;
  eye_d_to_world[0][0] = ex[0];
  eye_d_to_world[0][1] = ex[1];
  eye_d_to_world[0][2] = ex[2];

  eye_d_to_world[1][0] = ey[0];
  eye_d_to_world[1][1] = ey[1];
  eye_d_to_world[1][2] = ey[2];

  eye_d_to_world[2][0] = ez[0];
  eye_d_to_world[2][1] = ez[1];
  eye_d_to_world[2][2] = ez[2];

  eye_d_to_world[3][0] = origin[0];
  eye_d_to_world[3][1] = origin[1];
  eye_d_to_world[3][2] = origin[2];

  eye_d_to_world = glm::inverse(eye_d_to_world);
  
  return chessboard_pose * eye_d_to_world;

}
