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

  std::string pose_offset_filename = "./poseoffset";
  float tracking_offset_time = 0.0; // in seconds
  float color_offset_time = 0.0;
  bool append_samples = false;
  bool undistort = false;
  CMDParser p("calibbasefilename sweepfilename samplesfilename");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("t",1,"trackingoffset", "offset in seconds of the tracking system relative to depth frame of the sensor, e.g. 0.08, default: 0.0");
  p.addOpt("c",1,"coloroffset", "offset in seconds of the color frame relative to the depth frame of the sensor , e.g. -0.02, default: 0.0");
  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");
  p.init(argc,argv);

  if(p.getArgs().size() != 3){
    p.showHelp();
  }

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }
  if(p.isOptSet("t")){
    tracking_offset_time = p.getOptsFloat("t")[0];
  }
  if(p.isOptSet("c")){
    color_offset_time = p.getOptsFloat("c")[0];
  }
  if(p.isOptSet("u")){
    undistort = true;
  }

  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  const std::string filename_yml(basefilename + "_yml");

  CalibVolume cv(filename_xyz.c_str(), filename_uv.c_str());

  RGBDConfig cfg;
  cfg.read(filename_yml.c_str());

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cs(p.getArgs()[1].c_str(), cfg, undistort);
  cs.init();
  //cs.dump();

  SweepSampler ss(&cb, &cv, &cfg);
  const size_t numsamples = ss.extractSamples(&cs, tracking_offset_time, color_offset_time);
  ss.appendSamplesToFile(p.getArgs()[2].c_str(), append_samples);


  return 0;
}
