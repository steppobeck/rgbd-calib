#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <stablesampler.hpp>
#include <calibrator.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){
  bool using_nni = false;
  CMDParser p("calibvolume_xyz calibvolume_uv groundtruthsamplesfile");
  p.addOpt("i",-1,"nni", "calibration was performed using natural neighbor interpolation, default: false");
  p.init(argc,argv);



  if(p.getArgs().size() != 3)
    p.showHelp();

  if(p.isOptSet("i")){
    using_nni = true;
  }


  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  std::string filename_xyz(p.getArgs()[0]);
  std::string filename_uv(p.getArgs()[1]);

  // load samples from filename
  std::vector<samplePoint> sps;

  std::ifstream iff(p.getArgs()[2].c_str(), std::ifstream::binary);
  const unsigned num_samples_in_file = calcNumFrames(iff,
						     sizeof(float) +
						     sizeof(uv) +
						     sizeof(uv) +
						     sizeof(xyz) +
						     sizeof(uv) +
						     sizeof(glm::vec3) +
						     sizeof(float));
  for(unsigned i = 0; i < num_samples_in_file; ++i){
    samplePoint s;
    iff.read((char*) &s.depth, sizeof(float));
    iff.read((char*) &s.tex_color, sizeof(uv));
    iff.read((char*) &s.tex_depth, sizeof(uv));
    iff.read((char*) &s.pos_offset, sizeof(xyz));
    iff.read((char*) &s.tex_offset, sizeof(uv));
    iff.read((char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
    iff.read((char*) &s.quality, sizeof(float));
    sps.push_back(s);
  }
  iff.close();  


  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());
  Calibrator   c;
  c.evaluateSamples(&cv, sps, cfg, filename_xyz.c_str() , using_nni);

  return 0;
}
