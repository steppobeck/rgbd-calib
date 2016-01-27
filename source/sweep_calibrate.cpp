#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <sweepsampler.hpp>
#include <calibrator.hpp>

#include <fstream>
#include <iostream>


int main(int argc, char* argv[]){

  std::string pose_offset_filename = "../../../source/poseoffset";
  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 256;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 4.5;


  // ./sweep_sampler ../../../data/23.cv ../../../data/23_sweep -t -0.157 -c -0.01 -n

  const float tracking_offset_time_step = 0.01; // in seconds
  float tracking_offset_time_min = -0.2; // in seconds
  float tracking_offset_time_max = 0.0; // in seconds
  float color_offset_time = -0.01;

  unsigned idwneighbours = 20;

  bool reload = true;

  CMDParser p("calibvolumebasefilename checkerboardview_init checkerboardview_sweep");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename where to store the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.5");

  p.addOpt("t",2,"trackingoffsetrange", "min and max offset in seconds of the tracking system relative to depth frame of the sensor, e.g. -0.05 0.03, default: -0.2 0.0");
  p.addOpt("c",1,"coloroffset", "offset in seconds of the color frame relative to the depth frame of the sensor , e.g. -0.02, default: -0.01");

  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 20");

  p.addOpt("r",-1,"noreload", "do not reload sweep file from disk, default: reload = true");


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


  if(p.isOptSet("t")){
    tracking_offset_time_min = p.getOptsFloat("t")[0];
    tracking_offset_time_max = p.getOptsFloat("t")[1];
  }
  if(p.isOptSet("c")){
    color_offset_time = p.getOptsFloat("c")[0];
  }

  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("r")){
    reload = false;
  }

  CalibVolume cv_init(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);

  cfg.principal_rgb = glm::vec2(701.972473, 532.143066);
  cfg.principal_d   = glm::vec2(257.009552, 209.077789);

  cfg.focal_rgb = glm::vec2(1030.829834, 1030.497070);
  cfg.focal_d   = glm::vec2(355.433716, 355.672363);

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


  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cbs(p.getArgs()[1].c_str());
  cbs.init(reload);

  glm::mat4 eye_d_to_world = sensor.guess_eye_d_to_world_static(cbs, cb);
  std::cerr << "extrinsic of sensor is: " << eye_d_to_world << std::endl;
  std::cerr << "PLEASE note, the extrinsic guess can be improved by averaging" << std::endl;

  for(unsigned z = 0; z < cv_init.depth; ++z){
    for(unsigned y = 0; y < cv_init.height; ++y){
      for(unsigned x = 0; x < cv_init.width; ++x){

	const unsigned cv_index = (z * cv_init.width * cv_init.height) + (y * cv_init.width) + x;

	const float depth = (z + 0.5) * (cv_init.max_d - cv_init.min_d)/cv_init.depth + cv_init.min_d;
	const float xd = (x + 0.5) * sensor.config.size_d.x * 1.0/cv_init.width;
	const float yd = (y + 0.5) * sensor.config.size_d.y * 1.0/cv_init.height;

	glm::vec3 pos3D_local = sensor.calc_pos_d(xd, yd, depth);
	glm::vec2 pos2D_rgb   = sensor.calc_pos_rgb(pos3D_local);
	pos2D_rgb.x /= sensor.config.size_rgb.x;
	pos2D_rgb.y /= sensor.config.size_rgb.y;

	glm::vec4 pos3D_world = eye_d_to_world * glm::vec4(pos3D_local.x, pos3D_local.y, pos3D_local.z, 1.0);

	xyz pos3D;
	pos3D.x = pos3D_world.x;
	pos3D.y = pos3D_world.y;
	pos3D.z = pos3D_world.z;
	cv_init.cv_xyz[cv_index] = pos3D;

	uv posUV;
	posUV.u = pos2D_rgb.x;
	posUV.v = pos2D_rgb.y;
	cv_init.cv_uv[cv_index] = posUV;
      }
    }
  }


  const std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());

  // 2. load recording from sweepfilename
  ChessboardSampling cs_sweep(p.getArgs()[2].c_str());
  cs_sweep.init(reload);


  // 3. optimize tracking_offset_time
  float best_avg_quality = 0.0f;
  float best_tracking_offset_time = 0.0f;
  for(float tracking_offset_time = tracking_offset_time_min;
      tracking_offset_time < tracking_offset_time_max;
      tracking_offset_time += tracking_offset_time_step){

    CalibVolume cv_sweep(filename_xyz.c_str(), filename_uv.c_str());

    cs_sweep.loadChessboards();

    SweepSampler ss(&cb, &cv_sweep, &cfg);
    const size_t numsamples = ss.extractSamples(&cs_sweep, tracking_offset_time, color_offset_time);

    const std::vector<samplePoint>& sps = ss.getSamplePoints();
    Calibrator   c;
    c.applySamples(&cv_sweep, sps, cfg, idwneighbours);
  
    // evalute at valid chessboard location
    const float avg_quality = c.evalutePlanes(&cv_sweep, &cs_sweep, cfg);

    if(avg_quality > best_avg_quality){
      best_avg_quality = avg_quality;
      best_tracking_offset_time = tracking_offset_time;
    }

    std::cout << "tracking_offset_time: " << tracking_offset_time << " (best: " << best_tracking_offset_time << ") -> avg_quality: " << avg_quality << " (best: " << best_avg_quality << ")" << std::endl;

  }

  std::cout << "best_tracking_offset_time: " << best_tracking_offset_time << std::endl;

  cs_sweep.loadChessboards();
  SweepSampler ss(&cb, &cv_init, &cfg);
  const size_t numsamples = ss.extractSamples(&cs_sweep, best_tracking_offset_time, color_offset_time);

  const std::vector<samplePoint>& sps = ss.getSamplePoints();
  Calibrator   c;
  c.applySamples(&cv_init, sps, cfg, idwneighbours);


  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());
  filename_xyz += "_sweep";
  filename_uv  += "_sweep";
  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());

  return 0;
}
