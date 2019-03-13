#ifndef LPCDATATYPES_HPP
#define LPCDATATYPES_HPP

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/transform.hpp>


#include <iostream>

struct surfel{
  glm::dvec3 p;
  glm::uvec3 c;
  glm::vec3  n;
  float      r;
};

extern std::ostream& operator << (std::ostream& o, const glm::dvec3& v);

glm::uvec3 DataPointToColor(float Value, float MinValue, float MaxValue);
glm::uvec3 DataPointToColorSeismic(float Value, float MinValue, float MaxValue);

#endif // #ifndef LPCDATATYPES_HPP
