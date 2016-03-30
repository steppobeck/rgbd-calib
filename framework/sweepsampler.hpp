#ifndef RGBD_CALIB_SWEEPSAMPLER_HPP
#define RGBD_CALIB_SWEEPSAMPLER_HPP

#include <DataTypes.hpp>
#include <ChessboardSampling.hpp>

class CalibVolume;
class RGBDConfig;

class SweepSampler{

public:

  SweepSampler(const Checkerboard* cb, const CalibVolume* cv, const RGBDConfig * cfg);
  ~SweepSampler();

  void dumpSamplePoints();
  const std::vector<samplePoint>& getSamplePoints();

  void appendSamplesToFile(const char* filename, bool append_samples);


  size_t extractSamples(ChessboardSampling* cbs, const float pose_offset, const float color_offset, unsigned stride = 1);


private:


  void addBoardSamples(const glm::mat4& cb_transform, const ChessboardViewIR* corners_ir, const ChessboardViewRGB* corners_rgb);
  std::vector<samplePoint> m_sps;
  const Checkerboard* m_cb;
  const CalibVolume* m_cv;
  const RGBDConfig * m_cfg;

};


#endif // #ifndef RGBD_CALIB_SWEEPSAMPLER_HPP

