#ifndef RGBD_CALIB_RGBDSENSOR_HPP
#define RGBD_CALIB_RGBDSENSOR_HPP

#include <ChessboardSampling.hpp>
#include <DataTypes.hpp>

#define GLM_FORCE_RADIANS
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <zmq.hpp>

#include <string>

class RGBDConfig{
public:
  glm::uvec2 size_rgb;
  glm::uvec2 size_d;

  glm::vec2 principal_rgb;
  glm::vec2 principal_d;

  glm::vec2 focal_rgb;
  glm::vec2 focal_d;

  glm::mat4 eye_d_to_eye_rgb;
  std::string serverport;
};


class RGBDSensor{

public:
  RGBDSensor(const RGBDConfig& cfg);
  ~RGBDSensor();

  RGBDConfig config;

  unsigned char* frame_rgb;
  unsigned char* frame_ir;
  float* frame_d;
  
  glm::vec3 calc_pos_d(float x /* in pixels*/, float y /*in pixels*/, float d /* in meters*/);
  // retrieve 2D pixel coordinates for a given 3D position in front of the sensor in pixels
  glm::vec2 calc_pos_rgb(const glm::vec3& pos_d);

  void recv(bool recvir = false);

  glm::vec3 get_rgb_bilinear_normalized(const glm::vec2& pos_rgb /*in pixels*/);

  glm::mat4 guess_eye_d_to_world(const ChessboardSampling& cbs, const Checkerboard& cb);

private:

  zmq::context_t m_ctx;
  zmq::socket_t  m_socket;

};


#endif // #ifndef RGBD_CALIB_RGBDSENSOR_HPP
