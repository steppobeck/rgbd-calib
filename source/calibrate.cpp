#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <calibrator.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){

  unsigned idwneighbours = 10;
  bool using_nni = false;

  CMDParser p("basefilename samplesfilename");
  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 10");

  p.addOpt("i",-1,"nni", "do use natural neighbor interpolation if possible, default: false");

  p.init(argc,argv);

  if(p.getArgs().size() != 2)
    p.showHelp();


  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
    std::cout << "setting to numneighbours " << idwneighbours << std::endl;
  }


  if(p.isOptSet("i")){
    using_nni = true;
  }

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  std::string filename_samples(p.getArgs()[1]);


  // load samples from filename
  std::vector<samplePoint> sps;

  std::ifstream iff(filename_samples.c_str(), std::ifstream::binary);
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
  c.using_nni = using_nni;
  c.applySamples(&cv, sps, cfg, idwneighbours, basefilename.c_str());
  cv.save(filename_xyz.c_str(), filename_uv.c_str());


  return 0;
}
