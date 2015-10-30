#ifndef RGBD_CALIB_POSE_TRACKER_HPP
#define RGBD_CALIB_POSE_TRACKER_H


#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <vector>


class PoseTracker{
  
public:
  PoseTracker();
  ~PoseTracker();
  
  void update(const glm::mat4& m);
  float getSpeed(float minspeed = 0.005f);
  bool isStable(const glm::mat4& m, float minspeed, unsigned numstables);
  float currSpeed(const glm::mat4& m);
  
private:
  
  
  std::vector<glm::mat4> _poses;
  unsigned _lastNumStables;
};


#endif // #ifndef  RGBD_CALIB_POSE_TRACKER_H
