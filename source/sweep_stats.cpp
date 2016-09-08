#include <ChessboardSampling.hpp>
#include <rgbdsensor.hpp>
#include <calibvolume.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <sweepsampler.hpp>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <iomanip>

int main(int argc, char* argv[]){

  std::string pose_offset_filename = "./poseoffset";
  float tracking_offset_time = 0.0; // in seconds
  float color_offset_time = 0.0;
  bool append_samples = false;
  bool undistort = false;
  CMDParser p("calibbasefilename sweepfilename logfilename");
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

  cs.filterSamples(tracking_offset_time);

#if 0
    unsigned input_frames;
    unsigned no_too_few_corners;
    unsigned flipped_boards;
    unsigned outliers;
    unsigned corrupt_depth;
    unsigned temporal_jitter;
    unsigned output_frames;
#endif

  std::ofstream off(p.getArgs()[2].c_str());
  off /*<< "input_frames "*/ << cs.p_sweep_stats.input_frames << std::endl;
  off /*<< "no_too_few_corners "*/ << cs.p_sweep_stats.no_too_few_corners << std::endl;
  off /*<< "flipped_boards "*/ << cs.p_sweep_stats.flipped_boards << std::endl;
  off /*<< "outliers "*/ << cs.p_sweep_stats.outliers << std::endl;
  off /*<< "corrupt_depth "*/ << cs.p_sweep_stats.corrupt_depth << std::endl;
  off /*<< "temporal_jitter "*/ << cs.p_sweep_stats.temporal_jitter << std::endl;
  off /*<< "output_frames "*/ << cs.p_sweep_stats.output_frames << std::endl;
  off /*<< "output_samples "*/ << cs.p_sweep_stats.output_frames * CB_WIDTH*CB_HEIGHT << std::endl;



  std::vector<double> speeds;

  std::vector<double> dense_x;
  std::vector<double> dense_y;
  std::vector<double> dense_z;

  const std::vector<ChessboardRange>& valid_ranges = cs.getValidRanges();
  const std::vector<ChessboardViewIR>& cb_irs = cs.getIRs();
  for(const auto& r : valid_ranges){
    for(unsigned i = r.start + 1; i < r.end; ++i){

      const double time_a = cb_irs[i - 1].time;
      const double time_b = cb_irs[i].time;
      
      bool valid_pose_a = false;
      bool valid_pose_b = false;

      glm::mat4 pose_a = cs.interpolatePose(time_a, valid_pose_a);
      glm::mat4 pose_b = cs.interpolatePose(time_b, valid_pose_b);
      if(valid_pose_a && valid_pose_b){
	glm::vec4 dist_h = (pose_b * glm::vec4(0.0,0.0,0.0,1.0)) - (pose_a * glm::vec4(0.0,0.0,0.0,1.0));
	const double cms_per_sec = 100.0 * glm::length(glm::vec3(dist_h[0], dist_h[1], dist_h[2])) / (time_b - time_a);
	speeds.push_back(cms_per_sec);


	xyz c0 =  cb_irs[i - 1].corners[0];
	xyz c1 =  cb_irs[i - 1].corners[1];
	xyz c7 =  cb_irs[i - 1].corners[7];

	xyz c0b = cb_irs[i].corners[0];
	
	const float x_a = cv.width  *  ( c0.x)/ cfg.size_d.x;
	const float y_a = cv.height *  ( c0.y)/ cfg.size_d.y;
	const float z_a = cv.depth  *  ( c0.z - cv.min_d)/(cv.max_d - cv.min_d);

	const float x_b = cv.width  *  ( c1.x)/ cfg.size_d.x;
	const float y_b = cv.height *  ( c7.y)/ cfg.size_d.y;
	const float z_b = cv.depth  *  ( c0b.z - cv.min_d)/(cv.max_d - cv.min_d);

	const double d_x = 1.0/(x_b - x_a);
	const double d_y = 1.0/(y_b - y_a);
	const double d_z = 1.0/std::abs(z_b - z_a);

	dense_x.push_back(d_x);
	dense_y.push_back(d_y);
	dense_z.push_back(d_z);


      }
    }
  }

  double mean_cms_per_sec, sd_cms_per_sec;
  calcMeanSD(speeds, mean_cms_per_sec, sd_cms_per_sec);


  double mean_d_x, sd_d_x;
  double mean_d_y, sd_d_y;
  double mean_d_z, sd_d_z;

  calcMeanSD(dense_x, mean_d_x, sd_d_x);
  calcMeanSD(dense_y, mean_d_y, sd_d_y);
  calcMeanSD(dense_z, mean_d_z, sd_d_z);
  
  off << std::setprecision(3);
  off /*<< "speed "*/ << mean_cms_per_sec << " (" << sd_cms_per_sec << ")" << std::endl;
  off /*<< "density_x "*/ << mean_d_y << " (" << sd_d_y << ")" << std::endl;
  off /*<< "density_y "*/ << mean_d_x << " (" << sd_d_x << ")" << std::endl;
  off /*<< "density_z "*/ << mean_d_z << " (" << sd_d_z << ")" << std::endl;

  

  return 0;
}
