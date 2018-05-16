#include <rgbdsensor.hpp>

int main(int argc, char** argv){


  if(argc != 2){
    std::cout << "usage: " << argv[0] << "filename.yml" << std::endl;
    return 0;
  }


  RGBDConfig cfg;
  cfg.read(argv[1]);

  cfg.dump();  

  return 0;
}


