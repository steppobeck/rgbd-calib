#ifndef RGBD_CALIB_CALIBVOLUME_HPP
#define RGBD_CALIB_CALIBVOLUME_HPP

#include <DataTypes.hpp>


#define GLM_FORCE_RADIANS
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

class CalibVolume{

public:
  CalibVolume(const char* filename_xyz, const char* filename_uv);
  CalibVolume(unsigned width, unsigned height, unsigned depth, float min_d, float max_d);
  ~CalibVolume();

  unsigned width;
  unsigned height;
  unsigned depth;
  float min_d;
  float max_d;

  xyz* cv_xyz;
  uv*  cv_uv;

  void save(const char* filename_xyz, const char* filename_uv);

  glm::vec3 lookupPos3D(float x_norm, float y_norm, float d);
  glm::vec2 lookupPos2D_normalized(float x_norm, float y_norm, float d);


private:

};


#endif // #ifndef RGBD_CALIB_CALIBVOLUME_HPP

