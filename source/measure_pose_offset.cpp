#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <ARTListener.hpp>

#include <iostream>
#include <string>
#include <unistd.h>

int main(int argc, char* argv[]){

  std::string pose_offset_filename = "../../../source/poseoffset";
  unsigned art_port = 5000;
  unsigned target_id = 6;
  unsigned correction_id = 8;
  CMDParser p("");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename where to store the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("a",1,"artport", "specify the port of the A.R.T. tracking system, default: 5000");
  p.addOpt("t",1,"targetid", "specify the id of the checkerboard target, default: 6");
  p.addOpt("c",1,"correctionid", "specify the id of the correction target, default: 8");

  p.init(argc,argv);

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }

  if(p.isOptSet("a")){
    art_port = p.getOptsInt("a")[0];
    std::cout << "setting artport to " << art_port << std::endl;
  }

  if(p.isOptSet("t")){
    target_id = p.getOptsInt("t")[0];
    std::cout << "setting targetid to " << target_id << std::endl;
  }

  if(p.isOptSet("c")){
    correction_id = p.getOptsInt("c")[0];
    std::cout << "setting correctionid to " << correction_id << std::endl;
  }




  ARTListener l;
  l.open(art_port);

  for(unsigned i = 0; i < 10; ++i){
    sleep(1);
    l.listen();
  }


  glm::mat4 chessboard_pose_correction(l.getMatrices()[correction_id]);
  glm::mat4 chessboard_pose(l.getMatrices()[target_id]);
  glm::mat4 offset = (chessboard_pose_correction * glm::inverse(chessboard_pose));
  Checkerboard cb;
  cb.pose_offset = offset;
  cb.save_pose_offset(pose_offset_filename.c_str());

  std::cerr << "PLEASE note, the calculation can be improved by averaging" << std::endl;

  return 0;
}
