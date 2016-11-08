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

  std::string pose_offset_filename = "./poseoffset";
  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 128;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 3.0;
  bool undistort = false;

  unsigned idwneighbours = 10;
  bool using_nni = false;

  CMDParser p("basefilename samplesfilename checkerboardviewinitfilename");

  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.5");
p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");

  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 10");

  p.addOpt("i",-1,"nni", "do use natural neighbor interpolation if possible, default: false");

  p.init(argc,argv);

  if(p.getArgs().size() != 3)
    p.showHelp();

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }


  if(p.isOptSet("s")){
    cv_width = p.getOptsInt("s")[0];
    cv_height = p.getOptsInt("s")[1];
    cv_depth = p.getOptsInt("s")[2];
  }

  if(p.isOptSet("d")){
    cv_min_d = p.getOptsInt("d")[0];
    cv_max_d = p.getOptsInt("d")[1];
  }
  if(p.isOptSet("u")){
    undistort = true;
}


  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
    std::cout << "setting to numneighbours " << idwneighbours << std::endl;
  }


  if(p.isOptSet("i")){
    using_nni = true;
  }



  std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  const std::string filename_yml(basefilename + "_yml");
  std::string filename_samples(p.getArgs()[1]);


  CalibVolume cv(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.read(filename_yml.c_str());

  RGBDSensor sensor(cfg);

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());


  ChessboardSampling cbs(p.getArgs()[2].c_str(), cfg, undistort);
  cbs.init();

  glm::mat4 eye_d_to_world = sensor.guess_eye_d_to_world_static(cbs, cb);
  std::cerr << "extrinsic of sensor is: " << eye_d_to_world << std::endl;
  std::cerr << "PLEASE note, the extrinsic guess can be improved by averaging" << std::endl;

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

	glm::vec4 pos3D_world = eye_d_to_world * glm::vec4(pos3D_local.x, pos3D_local.y, pos3D_local.z, 1.0);

	xyz pos3D;
	pos3D.x = pos3D_world.x;
	pos3D.y = pos3D_world.y;
	pos3D.z = pos3D_world.z;
	cv.cv_xyz[cv_index] = pos3D;

	uv posUV;
	posUV.u = pos2D_rgb.x;
	posUV.v = pos2D_rgb.y;
	cv.cv_uv[cv_index] = posUV;
      }
    }
  }


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


  // reapply correction offsets
  for(unsigned i = 0; i < sps.size(); ++i){
    const unsigned cv_width = cv.width;
    const unsigned cv_height = cv.height;
    const unsigned cv_depth = cv.depth;

    const float x = cv_width  *  ( sps[i].tex_depth.u) / cfg.size_d.x;
    const float y = cv_height *  ( sps[i].tex_depth.v)/ cfg.size_d.y;
    const float z = cv_depth  *  ( sps[i].depth - cv.min_d)/(cv.max_d - cv.min_d);

    xyz pos = getTrilinear(cv.cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
    uv  tex = getTrilinear(cv.cv_uv,  cv_width, cv_height, cv_depth, x , y , z );


    sps[i].pos_offset.x = sps[i].pos_real[0] - pos.x;
    sps[i].pos_offset.y = sps[i].pos_real[1] - pos.y;
    sps[i].pos_offset.z = sps[i].pos_real[2] - pos.z;
      
    sps[i].tex_offset.u = sps[i].tex_color.u/cfg.size_rgb.x - tex.u;
    sps[i].tex_offset.v = sps[i].tex_color.v/cfg.size_rgb.y - tex.v;

    sps[i].quality = 1.0f;
      
  }


  
  Calibrator   c;
  c.using_nni = using_nni;
  c.applySamples(&cv, sps, cfg, idwneighbours, basefilename.c_str());
  cv.save(filename_xyz.c_str(), filename_uv.c_str());


  return 0;
}
