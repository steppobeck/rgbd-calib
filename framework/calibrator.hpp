#ifndef RGBD_CALIB_CALIBRATOR_HPP
#define RGBD_CALIB_CALIBRATOR_HPP

#include <DataTypes.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <NearestNeighbourSearch.hpp>

class ChessboardSampling;
class NaturalNeighbourInterpolator;
class Calibrator{

public:
  static bool using_nni;

  Calibrator();
  ~Calibrator();

  void applySamples(CalibVolume* cv, const std::vector<samplePoint>& sps, const RGBDConfig& cfg, unsigned idwneighbours);
  void evaluateSamples(CalibVolume* cv, std::vector<samplePoint>& sps, const RGBDConfig& cfg);


  double evalutePlanes(CalibVolume* cv, ChessboardSampling* cbs, const RGBDConfig& cfg, unsigned stride = 1);

private:
  void applySamplesPerThread(CalibVolume* cv, const NearestNeighbourSearch* nns, unsigned tid, unsigned numthreads, unsigned idwneighbours, unsigned char* nni_possible, CalibVolume* cv_nni, const NaturalNeighbourInterpolator* nnip);
  void idw_interpolate(const std::vector<nniSample>& neighbours, unsigned idw_neigbours, nniSample& ipolant, const float max_influence_dist);





};


#endif // #ifndef  RGBD_CALIB_CALIBRATOR_HPP

