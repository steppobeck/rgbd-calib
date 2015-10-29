
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>

#include <fstream>
#include <iostream>


int main(int argc, char* argv[]){


  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 256;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 4.5;

  CMDParser p("basefilename");
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.0");
  
  p.init(argc,argv);

  if(p.isOptSet("s")){
    cv_width = p.getOptsInt("s")[0];
    cv_height = p.getOptsInt("s")[1];
    cv_depth = p.getOptsInt("s")[2];
  }

  if(p.isOptSet("d")){
    cv_min_d = p.getOptsInt("d")[0];
    cv_max_d = p.getOptsInt("d")[1];
  }

  std::string basefilename = p.getArgs()[0];

  CalibVolume cv(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);


  cfg.principal_rgb = glm::vec2(701.972, 532.143);
  cfg.principal_d   = glm::vec2(257.01, 209.078);

  cfg.focal_rgb = glm::vec2(1030.83, 1030.5);
  cfg.focal_d   = glm::vec2(355.434, 355.672);

  cfg.eye_d_to_eye_rgb[0][0] = 0.999950;
  cfg.eye_d_to_eye_rgb[0][1] = -0.009198;
  cfg.eye_d_to_eye_rgb[0][2] = -0.003908;
  cfg.eye_d_to_eye_rgb[0][3] = 0.0;

  cfg.eye_d_to_eye_rgb[1][0] = 0.009169;
  cfg.eye_d_to_eye_rgb[1][1] = 0.999932;
  cfg.eye_d_to_eye_rgb[1][2] = -0.007234;
  cfg.eye_d_to_eye_rgb[1][3] = 0.0;
 
  cfg.eye_d_to_eye_rgb[2][0] = 0.003974;
  cfg.eye_d_to_eye_rgb[2][1] = 0.007198;
  cfg.eye_d_to_eye_rgb[2][2] = 0.999966;
  cfg.eye_d_to_eye_rgb[2][3] = 0.0;

  cfg.eye_d_to_eye_rgb[3][0] = -0.051237;
  cfg.eye_d_to_eye_rgb[3][1] = 0.000667;
  cfg.eye_d_to_eye_rgb[3][2] = 0.000195;
  cfg.eye_d_to_eye_rgb[3][3] = 1.0;

  RGBDSensor sensor(cfg);

  for(unsigned z = 0; z < cv.depth; ++z){
    for(unsigned y = 0; y < cv.height; ++y){
      for(unsigned x = 0; x < cv.width; ++x){

	const unsigned cv_index = (z * cv.width * cv.height) + (y * cv.width) + x;

	const float depth = (z + 0.5) * (cv.max_d - cv.min_d)/cv.depth + cv.min_d;
	const float xd = (x + 0.5) * sensor.config.size_d.x * 1.0/cv.width;
	const float yd = (y + 0.5) * sensor.config.size_d.y * 1.0/cv.height;

	glm::vec3 pos3D_local = sensor.calc_pos_d(xd, yd, depth);
	glm::vec2 pos2D_rgb   = sensor.calc_pos_rgb(pos3D_local);
	pos2D_rgb.x /= sensor.config.size_rgb.x;
	pos2D_rgb.y /= sensor.config.size_rgb.y;

	xyz pos3D;
	pos3D.x = pos3D_local.x;
	pos3D.y = pos3D_local.y;
	pos3D.z = pos3D_local.z;
	cv.cv_xyz[cv_index] = pos3D;

	uv posUV;
	posUV.u = pos2D_rgb.x;
	posUV.v = pos2D_rgb.y;
	cv.cv_uv[cv_index] = posUV;
      }
    }
  }

  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  cv.save(filename_xyz.c_str(), filename_uv.c_str());

  return 0;
}
