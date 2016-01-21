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

  CMDParser p("calibvolumebasefilename groundtruthsamplesfile");

  p.init(argc,argv);
  if(p.getArgs().size() != 2)
    p.showHelp();


  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");


  // load samples from filename
  std::vector<samplePoint> sps;

  std::ifstream iff(p.getArgs()[1].c_str(), std::ifstream::binary);
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
  c.evaluateSamples(&cv, sps, cfg);

  return 0;
}
