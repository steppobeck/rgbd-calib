
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <stablesampler.hpp>
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

  Checkerboard cb;
  std::cerr << "please adjust " << cb.pose_offset << std::endl;
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }
  


  StableSampler ss(&sensor, &cv, 5000, 6, &cb);

#if 0
  // loop over frames and display color, depth and ir image
  while(true){
    ss.sample();
    // ssampler.getSamples -> put them into Calibframes
  }
#endif


  return 0;
}
