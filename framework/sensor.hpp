
#if !defined(SENSOR_SENSOR_H)

#define SENSOR_SENSOR_H

// includes, system
#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

// includes, project

#include <device.hpp>

// types, exported (class, enum, struct, union, typedef)


namespace sensor {

class sensor{
public:

  sensor(const device* /*device*/, size_t /*station_id*/);
  ~sensor();

  size_t                 getFrame() const;
  const timevalue& getTimestamp() const;
  const glm::mat4&     getMatrix();
  bool                   getButton(size_t /*which*/) const;

  void                   setTransmitterOffset(const glm::mat4& /*transmitterOffset*/);
  void                   setReceiverOffset(const glm::mat4& /*receiverOffset*/);

  const glm::mat4&     getTransmitterOffset() const;
  const glm::mat4&     getReceiverOffset() const;
  

private:
  
  const device::station& _station;
  glm::mat4      _devicemat;
  glm::mat4      _transmitterOffset;
  glm::mat4      _receiverOffset;
  

};

} // end namespace sensor {

// variables, exported (extern)

// functions, inlined (inline)
  
// functions, exported (extern)
  
#endif // #if !defined(SENSOR_SENSOR_H)

