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

  void applySamples(CalibVolume* cv, const std::vector<samplePoint>& sps, const RGBDConfig& cfg, unsigned idwneighbours, const char* basefilename,
		    RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0);
  void evaluateSamples(CalibVolume* cv, std::vector<samplePoint>& sps, const RGBDConfig& cfg, const char* basefilename, bool isnni);


  double evaluatePlanes(CalibVolume* cv, ChessboardSampling* cbs, const RGBDConfig& cfg, unsigned stride = 1);

  double evaluate3DError(CalibVolume* cv, ChessboardSampling* cbs, const Checkerboard* cb, const RGBDConfig& cfg, float delta_t_pose, unsigned stride = 1);

  double evaluate2DError(CalibVolume* cv, ChessboardSampling* cbs, const RGBDConfig& cfg, float delta_t_color, unsigned stride = 1);



  static void idw_interpolate(const std::vector<nniSample>& neighbours, const unsigned idw_neigbours, nniSample& ipolant, const float max_influence_dist);

private:
  void applySamplesPerThread(CalibVolume* cv, const NearestNeighbourSearch* nns, unsigned tid, unsigned numthreads, unsigned idwneighbours, CalibVolume* cv_nni, const NaturalNeighbourInterpolator* nnip);
  

  void blendIDW2NNI(CalibVolume* cv, CalibVolume* cv_nni, const char* basefilename);

  void createErrorVis(const std::vector<nniSample>& sps, const unsigned width, const unsigned height, const unsigned depth, const char* basefilename, bool isnni);
  void applyErrorVisPerThread(const unsigned width, const unsigned height, const unsigned depth,
			      unsigned char* error_vol_3D, unsigned char* error_vol_2D,
			      unsigned char* error_vol_3D_nni, unsigned char* error_vol_2D_nni,
			      const NearestNeighbourSearch* nns, const NaturalNeighbourInterpolator* nni,
			      const unsigned tid, const unsigned numthreads);

  unsigned char* m_nni_possible;



};


#endif // #ifndef  RGBD_CALIB_CALIBRATOR_HPP

