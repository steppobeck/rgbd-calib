#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>

int main(int argc, char* argv[]){

  std::string pose_offset_filename = "./poseoffset";
  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 128;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 3.0;
  bool undistort = false;

  std::ofstream* volume_log_x = 0;
  std::ofstream* volume_log_y = 0;
  std::ofstream* volume_log_z = 0;

  std::ofstream* volume_log_rgb = 0;
  std::ofstream* volume_log_depth = 0;

  CMDParser p("calibvolumebasefilename checkerboardviewinitfilename");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.5");
  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");
  p.addOpt("l",1, "logfile", "specify the prefix of the logfile for volume statistics, default: no logfiles are written");
  p.init(argc,argv);

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

  if(p.isOptSet("l")){
    // ./initialize ../../data/test/23.cv ../../data/test/23_init -l ../../data/test/23.volume_stats
    volume_log_x = new std::ofstream(p.getOptsString("l")[0] + "_x.csv");
    volume_log_y = new std::ofstream(p.getOptsString("l")[0] + "_y.csv");
    volume_log_z = new std::ofstream(p.getOptsString("l")[0] + "_z.csv");

    volume_log_rgb = new std::ofstream(p.getOptsString("l")[0] + "_rgb.csv");
    volume_log_depth = new std::ofstream(p.getOptsString("l")[0] + "_depth.csv");
  }
  // used for volume log files
  float* depth_at_voxel = new float[cv_width*cv_height*cv_depth];
  uv* rgb_cord_at_voxel = new uv[cv_width*cv_height*cv_depth];
  uv* depth_cord_at_voxel = new uv[cv_width*cv_height*cv_depth];


  const std::string basefilename = p.getArgs()[0];
  const std::string filename_yml(basefilename + "_yml");

  CalibVolume cv(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.read(filename_yml.c_str());

  RGBDSensor sensor(cfg);


  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());


  ChessboardSampling cbs(p.getArgs()[1].c_str(), cfg, undistort);
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

	// used for volume log files
	depth_at_voxel[cv_index] = depth;
	uv dcav;
	dcav.u = xd;
	dcav.v = yd;
	depth_cord_at_voxel[cv_index] = dcav;
	uv rgbcav;
	rgbcav.u = posUV.u * sensor.config.size_rgb.x;
	rgbcav.v = posUV.v * sensor.config.size_rgb.y;
	rgb_cord_at_voxel[cv_index] = rgbcav;

      }
    }
  }



  const std::string filename_xyz(basefilename + "_xyz");
  const std::string filename_uv(basefilename + "_uv");
  cv.save(filename_xyz.c_str(), filename_uv.c_str());

  

  if(volume_log_x != 0){

    for(unsigned z = 0; z < (cv.depth - 1); ++z){

      std::vector<float> avg_depths;
      std::vector<float> sizes_x;
      std::vector<float> sizes_y;
      std::vector<float> sizes_z;

      std::vector<float> sizes_rgb;
      std::vector<float> sizes_depth;

      for(unsigned y = 0; y < (cv.height - 1); ++y){
	for(unsigned x = 0; x < (cv.width - 1); ++x){

	  const unsigned cv_index = (z * cv.width * cv.height) + (y * cv.width) + x;
	  const unsigned cv_index_x = (z * cv.width * cv.height) + (y * cv.width) + (x + 1);
	  const unsigned cv_index_y = (z * cv.width * cv.height) + ((y + 1) * cv.width) + x;
	  const unsigned cv_index_z = ((z + 1) * cv.width * cv.height) + (y * cv.width) + x;

	  avg_depths.push_back(depth_at_voxel[cv_index]);

	  const xyz a = cv.cv_xyz[cv_index];
	  const glm::vec3 ga(a.x, a.y, a.z);

	  const xyz ax = cv.cv_xyz[cv_index_x];
	  const glm::vec3 gax(ax.x, ax.y, ax.z);

	  const xyz ay = cv.cv_xyz[cv_index_y];
	  const glm::vec3 gay(ay.x, ay.y, ay.z);

	  const xyz az = cv.cv_xyz[cv_index_z];
	  const glm::vec3 gaz(az.x, az.y, az.z);

	  sizes_x.push_back(glm::length(gax - ga));
	  sizes_y.push_back(glm::length(gay - ga));
	  sizes_z.push_back(glm::length(gaz - ga));

	  sizes_rgb.push_back( (rgb_cord_at_voxel[cv_index_x].u - rgb_cord_at_voxel[cv_index].u) *
			       (rgb_cord_at_voxel[cv_index_y].v - rgb_cord_at_voxel[cv_index].v));

	  sizes_depth.push_back( (depth_cord_at_voxel[cv_index_x].u - depth_cord_at_voxel[cv_index].u) *
				 (depth_cord_at_voxel[cv_index_y].v - depth_cord_at_voxel[cv_index].v));


	}
      }

      double mean_depth, sd_depth;
      calcMeanSD(avg_depths, mean_depth, sd_depth);

      double mean_x, sd_x;
      calcMeanSD(sizes_x, mean_x, sd_x);

      double mean_y, sd_y;
      calcMeanSD(sizes_y, mean_y, sd_y);

      double mean_z, sd_z;
      calcMeanSD(sizes_z, mean_z, sd_z);

      *volume_log_x << std::setprecision(3) << mean_depth << ", " << 1000 * mean_x << std::endl;
      *volume_log_y << std::setprecision(3) << mean_depth << ", " << 1000 * mean_y << std::endl;
      *volume_log_z << std::setprecision(3) << mean_depth << ", " << 1000 * mean_z << std::endl;


      double mean_rgb_cord, sd_rgb_cord;
      calcMeanSD(sizes_rgb, mean_rgb_cord, sd_rgb_cord);

      double mean_depth_cord, sd_depth_cord;
      calcMeanSD(sizes_depth, mean_depth_cord, sd_depth_cord);


      *volume_log_rgb << std::setprecision(3) << mean_depth << ", " << mean_rgb_cord << std::endl;
      *volume_log_depth << std::setprecision(3) << mean_depth << ", " << mean_depth_cord << std::endl;


    }



    delete volume_log_x;
    delete volume_log_y;
    delete volume_log_z;

  }

  delete [] depth_at_voxel;

  delete [] depth_cord_at_voxel;
  delete [] rgb_cord_at_voxel;

  return 0;
}
