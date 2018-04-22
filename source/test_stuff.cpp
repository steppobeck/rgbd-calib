#include <image.hpp>

int main(int argc, char** argv){

  png::image< png::rgb_pixel > output(640,480);
  for(unsigned int y = 0; y < 480; ++y){
    for(unsigned int x = 0; x < 640; ++x){
      png::rgb_pixel p;
      p.red   = 255;
      p.green = 0;
      p.blue  = 0;
      output.set_pixel(x, y, p);
    }
  }
  output.write("/tmp/test_png.png");
  return 0;
}


