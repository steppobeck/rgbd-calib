#ifndef RGBD_CALIB_ARTLISTENER_HPP
#define RGBD_CALIB_ARTLISTENER_HPP

#define ARTLISTENERNUMSENSORS 50
#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

#include <timevalue.hpp>

#include <vector>
#include <map>
#include <string>

namespace sensor{
  class sensor;
  class device;
}


  class ChronoMeter;
  class ARTListener{

  public:
    ARTListener();
    ~ARTListener();

    bool isOpen();
    bool open(unsigned port, const char* filename = 0, unsigned * id = 0, ChronoMeter* cm = 0);
    glm::mat4 listen(unsigned id);
    void listen();
    bool close();
    void fill(void* destination);
    std::vector<glm::mat4>& get(const void* source);
    const std::vector<glm::mat4>& getMatrices() const;

  private:

    std::vector<glm::mat4> m_matrices;
    std::vector<sensor::sensor*> m_sensors;
    sensor::device* m_dev;

  };





#endif  // #ifndef RGBD_CALIB_ARTLISTENER_H

