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


  CMDParser p("sweepfilename");
  unsigned start = 0;
  unsigned end = 0;
  p.addOpt("r",2,"range", "show this range only, e.g. -r 300 320 default: show all");

  p.init(argc,argv);

  if(p.isOptSet("r")){
    start = p.getOptsInt("r")[0];
    end = p.getOptsInt("r")[1];
  }


  ChessboardSampling cs(p.getArgs()[0].c_str());
  cs.interactiveShow(start, end);



  return 0;
}
