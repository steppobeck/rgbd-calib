#include "rgbdsensor.hpp"




#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>


#include <cmath>
#include <iostream>


namespace{

  unsigned char* convertTo8Bit(float* in, unsigned w, unsigned h){
    static unsigned char* b = new unsigned char [w*h];

    float max_v = 0.0;
    for(unsigned idx = 0; idx != w*h; ++idx){
      max_v = std::max(in[idx],max_v);
    }
    if(max_v != 0.0){
      for(unsigned idx = 0; idx != w*h; ++idx){
	const float n = in[idx]/max_v;
	unsigned char v = (unsigned char) (255.0 * n);
	b[idx] = v;
      }
    }

    return b;
  }

}

RGBDSensor::RGBDSensor(const RGBDConfig& cfg, unsigned num_of_slaves)
  : config(cfg),
    frame_rgb(0),
    frame_ir(0),
    frame_d(0),
    slave_frames_rgb(),
    slave_frames_d(),
    num_slaves(num_of_slaves),
    m_ctx(1),
    m_socket(m_ctx, ZMQ_SUB)
{

  frame_rgb = new unsigned char [3 * config.size_rgb.x * config.size_rgb.y];
  frame_ir  = new unsigned char [config.size_d.x * config.size_d.y];
  frame_d   = new float         [config.size_d.x * config.size_d.y];
  
  
  for(unsigned i = 0; i < num_slaves; ++i){
    slave_frames_rgb.push_back(new unsigned char [3 * config.size_rgb.x * config.size_rgb.y]);
    slave_frames_d.push_back(new float         [config.size_d.x * config.size_d.y]);
  }


  if(config.serverport != ""){
    m_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

#if ZMQ_VERSION_MAJOR < 3
    uint64_t hwm = 1;
    m_socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
    uint32_t hwm = 1;
    m_socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
#endif
    std::string endpoint("tcp://" + cfg.serverport);
    //std::cout << "opening socket connection to: " << endpoint << std::endl;
    m_socket.connect(endpoint.c_str());
  }


  cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("depth", CV_WINDOW_AUTOSIZE);
  m_cv_rgb_image = cvCreateImage(cvSize(config.size_rgb.x, config.size_rgb.y), 8, 3);
  m_cv_depth_image = cvCreateImage(cvSize(config.size_d.x, config.size_d.y), 8, 1);

}



RGBDSensor::~RGBDSensor(){
  delete [] frame_rgb;
  delete [] frame_ir;
  delete [] frame_d;

  cvReleaseImage(&m_cv_rgb_image);
  cvReleaseImage(&m_cv_depth_image);

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
  zmq::message_t zmqm(bytes_recv + num_slaves * (bytes_rgb + bytes_d));
  m_socket.recv(&zmqm); // blocking
  unsigned offset = 0;
  memcpy((unsigned char*) frame_rgb, (unsigned char*) zmqm.data() + offset, bytes_rgb);
  offset += bytes_rgb;
  memcpy((unsigned char*) frame_d, (unsigned char*) zmqm.data() + offset, bytes_d);
  offset += bytes_d;
  if(recvir){
    memcpy((unsigned char*) frame_ir, (unsigned char*) zmqm.data() + offset, bytes_ir);
  }
  for(unsigned i = 0; i < num_slaves; ++i){
    memcpy((unsigned char*) slave_frames_rgb[i], (unsigned char*) zmqm.data() + offset, bytes_rgb);
    offset += bytes_rgb;
    memcpy((unsigned char*) slave_frames_d[i], (unsigned char*) zmqm.data() + offset, bytes_d);
    offset += bytes_d;
  }






}

void
RGBDSensor::display_rgb_d(){
  const unsigned bytes_rgb(3 * config.size_rgb.x * config.size_rgb.y);
  const unsigned bytes_ir(config.size_d.x * config.size_d.y);

  IplImage* tmp_image = cvCreateImage(cvSize(config.size_rgb.x, config.size_rgb.y), 8, 3);
  memcpy(m_cv_rgb_image->imageData, frame_rgb, bytes_rgb);
  cvCvtColor( m_cv_rgb_image, tmp_image, CV_BGR2RGB );
  cvShowImage( "rgb", tmp_image);
  cvReleaseImage(&tmp_image);
  memcpy(m_cv_depth_image->imageData, convertTo8Bit(frame_d, config.size_d.x, config.size_d.y), bytes_ir);
  cvShowImage( "depth", m_cv_depth_image);
  int key = cvWaitKey(10);

}

glm::vec3
RGBDSensor::get_rgb_bilinear_normalized(const glm::vec2& pos_rgb, unsigned stream_num){

  glm::vec3 rgb;


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


  unsigned char* frame_rgb_real = stream_num == 0 ? frame_rgb : slave_frames_rgb[stream_num - 1];

  // calculate indices to access data
  const unsigned idmax = 3u * config.size_rgb.x * config.size_rgb.y - 2u;
  const unsigned id00 = std::min( ya * 3u * config.size_rgb.x + 3u * xa  , idmax);
  const unsigned id10 = std::min( ya * 3u * config.size_rgb.x + 3u * xb  , idmax);
  const unsigned id01 = std::min( yb * 3u * config.size_rgb.x + 3u * xa  , idmax);
  const unsigned id11 = std::min( yb * 3u * config.size_rgb.x + 3u * xb  , idmax);
  

  // RED CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb_real[id00] + w_xb * frame_rgb_real[id10];
    const float tmp_yb = w_xa * frame_rgb_real[id01] + w_xb * frame_rgb_real[id11];
    // 2. interpolate between y direction;
    rgb.x = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  // GREEN CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb_real[id00 + 1] + w_xb * frame_rgb_real[id10 + 1];
    const float tmp_yb = w_xa * frame_rgb_real[id01 + 1] + w_xb * frame_rgb_real[id11 + 1];
    // 2. interpolate between y direction;
    rgb.y = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  // BLUE CHANNEL
  {
    // 1. interpolate between x direction;
    const float tmp_ya = w_xa * frame_rgb_real[id00 + 2] + w_xb * frame_rgb_real[id10 + 2];
    const float tmp_yb = w_xa * frame_rgb_real[id01 + 2] + w_xb * frame_rgb_real[id11 + 2];
    // 2. interpolate between y direction;
    rgb.z = w_ya * tmp_ya + w_yb * tmp_yb;
  }

  rgb.x /= 255.0;
  rgb.y /= 255.0;
  rgb.z /= 255.0;

  return rgb;

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
