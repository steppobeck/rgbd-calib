#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <stablesampler.hpp>
#include <calibrator.hpp>
#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){
  unsigned stride = 0;
  unsigned idwneighbours = 20;
  CMDParser p("basefilename");
  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 20");
  p.addOpt("s",1,"stride", "the stride to use for evaluation, default: 0 (no evaluation performed)");

  p.init(argc,argv);

  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("s")){
    stride = p.getOptsInt("s")[0];
  }

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  std::string filename_samples(basefilename + "_samples");

  // load samples



  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());

  Calibrator   c;
  c.applySamples(&cv, filename_samples.c_str(), cfg, idwneighbours);

  cv.save(filename_xyz.c_str(), filename_uv.c_str());
  


  return 0;
}
