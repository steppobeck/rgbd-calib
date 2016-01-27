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
  unsigned evaluation_stride = 0;
  unsigned idwneighbours = 20;
  CMDParser p("basefilename");
  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 20");
  p.addOpt("e",1,"evaluationstride", "the stride to use for evaluation, default: 0 (no evaluation performed)");

  p.init(argc,argv);

  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
    std::cout << "setting to numneighbours " << idwneighbours << std::endl;
  }

  if(p.isOptSet("e")){
    evaluation_stride = p.getOptsInt("e")[0];
    std::cout << "setting evaluationstride to " << evaluation_stride << std::endl;
  }

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  std::string filename_samples(basefilename + "_samples");


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

  if(evaluation_stride){
    std::vector<samplePoint> sps_calib;
    std::vector<samplePoint> sps_eval;
    unsigned sid = 0;
    bool to_calib = false;
    for(const auto& s : sps){
      if(sid % evaluation_stride == 0){
	to_calib = !to_calib;
      }
      
      to_calib ? sps_calib.push_back(s) : sps_eval.push_back(s);

      ++sid;
    }
      CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());
      Calibrator   c;
      std::cout << "--------------------------------------------------------------------" << std::endl;
      std::cout << "Calibration error before calibration at " << sps_eval.size() << " sample points" << std::endl;
      c.evaluateSamples(&cv, sps_eval, cfg);
      c.applySamples(&cv, sps_calib, cfg, idwneighbours);
      std::cout << "Calibration error after calibration at " << sps_eval.size() << " sample points" << std::endl;
      c.evaluateSamples(&cv, sps_eval, cfg);
      std::cout << "--------------------------------------------------------------------" << std::endl;
  }

  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());
  Calibrator   c;
  c.applySamples(&cv, sps, cfg, idwneighbours);
  cv.save(filename_xyz.c_str(), filename_uv.c_str());


  return 0;
}
