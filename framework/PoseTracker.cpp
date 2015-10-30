#include "PoseTracker.hpp"

#include <glm/vec3.hpp>

PoseTracker::PoseTracker()
  : _poses(),
    _lastNumStables(0)
{
  glm::mat4 m;
  _poses.push_back(m);
  _poses.push_back(m);
}



PoseTracker::~PoseTracker()
{}


void
PoseTracker::update(const glm::mat4& m){
  _poses[0] = _poses[1];
  _poses[1] = m;
  
}


float
PoseTracker::currSpeed(const glm::mat4& m){
  update(m);
  glm::vec3 a(_poses[0][3][0], _poses[0][3][1], _poses[0][3][2]);
  glm::vec3 b(_poses[1][3][0], _poses[1][3][1], _poses[1][3][2]);
  float speed = glm::length(b-a);
  return speed;
}


float
PoseTracker::getSpeed(float minspeed){
  glm::vec3 a(_poses[0][3][0], _poses[0][3][1], _poses[0][3][2]);
  glm::vec3 b(_poses[1][3][0], _poses[1][3][1], _poses[1][3][2]);
  float speed = glm::length(b-a);
  return speed < minspeed ? 0.0 : speed;
}

bool
PoseTracker::isStable(const glm::mat4& m, float minspeed, unsigned numstables){
  update(m);
  if(getSpeed(minspeed) < minspeed){
    ++_lastNumStables;
  }
  else{
    _lastNumStables = 0;
  }
  return (_lastNumStables > numstables);
}

