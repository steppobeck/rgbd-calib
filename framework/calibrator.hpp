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
  void applySamplesPerThread(CalibVolume* cv, const NearestNeighbourSearch* nns, unsigned tid, unsigned numthreads, unsigned idwneighbours, CalibVolume* cv_nni, const NaturalNeighbourInterpolator* nnip);
  void idw_interpolate(const std::vector<nniSample>& neighbours, unsigned idw_neigbours, nniSample& ipolant, const float max_influence_dist);

  void blendIDW2NNI(CalibVolume* cv, CalibVolume* cv_nni);

  unsigned char* m_nni_possible;



};


#endif // #ifndef  RGBD_CALIB_CALIBRATOR_HPP

