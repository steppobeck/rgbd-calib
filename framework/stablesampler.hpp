#ifndef RGBD_CALIB_STABLESAMPLER_HPP
#define RGBD_CALIB_STABLESAMPLER_HPP

#include <DataTypes.hpp>

class OpenCVChessboardCornerDetector;
class CalibVolume;
class ARTListener;
class RGBDSensor;
class StableSampler{

public:

  StableSampler(RGBDSensor* sensor, CalibVolume* cv, unsigned art_port, unsigned art_target_id, Checkerboard* cb);

  void sampleBoardLocation(float max_shaking_speed, unsigned min_num_frames_below_max_shaking, unsigned num_frames_to_filter);

  void dumpSamplePoints();
  const std::vector<samplePoint>& getSamplePoints();

  void appendSamplesToFile(const char* filename);


private:
  RGBDSensor* m_sensor;
  CalibVolume* m_cv;
  ARTListener* m_artl;
  unsigned m_art_target_id;
  OpenCVChessboardCornerDetector* m_cd_c;
  OpenCVChessboardCornerDetector* m_cd_i;
  std::vector<samplePoint> m_sps;
  Checkerboard* m_cb;
};



#endif // #ifndef RGBD_CALIB_STABLESAMPLER_HPP
