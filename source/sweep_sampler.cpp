#include <ChessboardSampling.hpp>
#include <rgbdsensor.hpp>
#include <calibvolume.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <sweepsampler.hpp>
#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){

  std::string pose_offset_filename = "../../../source/poseoffset";
  float pose_offset = 0.0; // in seconds
  float color_offset = 0.0;
  
  CMDParser p("calibbasefilename sweepfilename");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename where to store the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("t",1,"trackingposeoffset", "pose offset in seconds of the tarcking system e.g. 0.08, default 0.0");
  p.addOpt("c",1,"coloroffset", "color offset in seconds e.g. -0.02, default 0.0");

  p.init(argc,argv);

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }
  if(p.isOptSet("t")){
    pose_offset = p.getOptsFloat("t")[0];
  }
  if(p.isOptSet("c")){
    color_offset = p.getOptsFloat("c")[0];
  }


  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");

  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cs(p.getArgs()[1].c_str());
  cs.init(true);
  //cs.dump();

  SweepSampler ss(&cb, &cv, &cfg);
  const size_t numsamples = ss.extractSamples(&cs, pose_offset, color_offset);
  std::string filename_samples(basefilename + "_samples");
  ss.appendSamplesToFile(filename_samples.c_str());
  


  return 0;
}
