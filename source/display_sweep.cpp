#include <CMDParser.hpp>

#include <ChessboardSampling.hpp>



#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char* argv[]){


  CMDParser p("sweepfilename ymlfile");
  unsigned start = 0;
  unsigned end = 0;
  bool undistort = false;
  p.addOpt("r",2,"range", "show this range only, e.g. -r 300 320 default: show all");
  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");
  p.init(argc,argv);

  if(p.getArgs().size() != 2){
    p.showHelp();
  }

  if(p.isOptSet("r")){
    start = p.getOptsInt("r")[0];
    end = p.getOptsInt("r")[1];
  }
  if(p.isOptSet("u")){
    undistort = true;
  }


  RGBDConfig cfg;
  cfg.read(p.getArgs()[1].c_str());

  ChessboardSampling cs(p.getArgs()[0].c_str(), cfg, undistort);
  cs.interactiveShow(start, end);



  return 0;
}
