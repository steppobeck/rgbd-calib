#ifndef RGBD_CALIB_RESAMPLER_HPP
#define RGBD_CALIB_RESAMPLER_HPP

#include <NaturalNeighbourInterpolator.hpp>


#include <vector>

class CalibVolume;
class NearestNeighbourSearch;

class Resampler{
  
public:
  Resampler();
  ~Resampler();
  
  void resampleGridBased(std::vector<nniSample>& sps, const CalibVolume* cv, const char* basefilename);
  void fillBorder(std::vector<nniSample>& sps, const CalibVolume* cv, const NearestNeighbourSearch* nns, const unsigned idwneighbours, const char* basefilename);

};


#endif // #ifndef RGBD_CALIB_RESAMPLER_HPP
