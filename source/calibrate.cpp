
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <ChessboardSampling.hpp>
#include <fstream>
#include <iostream>


int main(int argc, char* argv[]){


  CMDParser p("basefilename serverport");
  p.init(argc,argv);

  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);
  cfg.serverport = p.getArgs()[1];
  RGBDSensor sensor(cfg);

  ChessboardSampling cs("/mnt/pitoti/tmp_steppo/23_sweep");
  cs.init(true);
  cs.dump();

#if 0
  //StableSampler ssampler(sensor)

  // loop over frames and display color, depth and ir image
  while(true){
    //ssampler.sample();
    // ssampler.getSamples -> put them into Calibframes
  }
#endif


  return 0;
}
