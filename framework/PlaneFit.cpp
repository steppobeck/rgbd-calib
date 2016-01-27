#include "PlaneFit.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>

namespace {

  template <class T>
  inline std::string
  toString(T value){
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }


}

float
detectPlaneQuality(const std::vector<xyz>& corners){

  std::string estr("/home/steppo/Desktop/my-git/rgbd-calib/thirdparty/planefit/planefit");
  for(const auto& c : corners){
    estr += " " + toString(c.x) + " " + toString(c.y) + " " + toString(c.z);
  }
  //std::cout << "execution of: " << estr  << std::endl;

  system(estr.c_str());

  float quality;
  std::ifstream qualityfile("/tmp/planequality");
  qualityfile >> quality;
  qualityfile.close();

  //std::cout << "quality is: " << quality  << std::endl;
  
  return quality;
}
