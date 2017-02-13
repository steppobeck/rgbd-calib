#include <CMDParser.hpp>

#include <ChessboardSampling.hpp>



#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){

  std::string pose_offset_filename = "./poseoffset";
  CMDParser p("sweepfilename ymlfile");
  unsigned start = 0;
  unsigned end = 0;
  bool undistort = false;
  bool try_detect = true;
  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("r",2,"range", "show this range only, e.g. -r 300 320 default: show all");
  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: undistortion disabled");
  p.addOpt("n", -1, "nodetect", "disable detecting of crossing points, default: decting crossing points enabled");
  p.init(argc,argv);

  if(p.getArgs().size() != 2){
    p.showHelp();
  }

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }

  if(p.isOptSet("r")){
    start = p.getOptsInt("r")[0];
    end = p.getOptsInt("r")[1];
  }

  if(p.isOptSet("u")){
    undistort = true;
  }
  if(p.isOptSet("n")){
    try_detect = false;
  }

  RGBDConfig cfg;
  cfg.read(p.getArgs()[1].c_str());

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }


  ChessboardSampling cs(p.getArgs()[0].c_str(), cfg, undistort);
  cs.interactiveShow(start, end, cb, try_detect);


  return 0;
}
