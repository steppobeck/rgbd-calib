#include <rgbdsensor.hpp>
#include <ChessboardSampling.hpp>
#include <iostream>

int main(int argc, char** argv){



  RGBDConfig cfg;
  cfg.read("/mnt/pitoti/kinect_recordings/cbsampling_test/23.cv_yml");

  ChessboardSampling cbs("/mnt/pitoti/kinect_recordings/cbsampling_test/23_sweep_out_Bottom", cfg, /*undistort*/ false);
  cbs.init();

  for(unsigned i = 0; i < 300; ++i){
    cbs.detectShapeFaults(i);
  }

  cbs.dump();

  return 0;
}

