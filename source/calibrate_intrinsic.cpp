#include <CMDParser.hpp>
#include <ChessboardSampling.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <OpenCVHelper.hpp>


void
extract_corners(ChessboardSampling* cbs, const std::vector<unsigned>& cb_ids,
		std::vector< std::vector<cv::Point2f> >& color_corners,
		std::vector< std::vector<cv::Point2f> >& depth_corners){


  std::cerr << "extracing corners from " << cb_ids.size() << " cb_ids" << std::endl;

  const std::vector<ChessboardViewIR>& cb_irs = cbs->getIRs();
  const std::vector<ChessboardViewRGB>& cb_rgbs = cbs->getRGBs();
  for(const auto& i : cb_ids){
      
    // add 35 corners to stereo_corners of cb_rgb_i
    std::vector<cv::Point2f> corners_rgb;
    std::vector<cv::Point2f> corners_d;
    for(unsigned idx = 0; idx < (CB_WIDTH * CB_HEIGHT); ++idx){
      corners_rgb.push_back(cv::Point2f(cb_rgbs[i].corners[idx].u,cb_rgbs[i].corners[idx].v));
      corners_d.push_back(cv::Point2f(cb_irs[i].corners[idx].x,cb_irs[i].corners[idx].y));
    }
    color_corners.push_back(corners_rgb);
    depth_corners.push_back(corners_d);
    
  }

  std::cerr << "resulting in " << color_corners.size() << " depth and color corners" << std::endl;
}

double
calibrate_camera(const std::vector< std::vector<cv::Point2f> >& corners,
		 const glm::uvec2& frame_size,
		 cv::Mat1d& intrinsics, cv::Mat1d& distortion, bool compute_distortions){

  const cv::Size image_size(frame_size.x, frame_size.y);
  std::vector< std::vector<cv::Point3f> > pattern_points;
  calibrationPattern(pattern_points,
		     CB_WIDTH,  CB_HEIGHT, 0.075,
		     corners.size());



  std::vector<cv::Mat> rvecs, tvecs;
  const double error = cv::calibrateCamera(pattern_points, corners, image_size,
					   intrinsics, distortion,
					   rvecs, tvecs,
					   compute_distortions ? 0 : CV_CALIB_ZERO_TANGENT_DIST |
					   CV_CALIB_FIX_K1 |
					   CV_CALIB_FIX_K2 |
					   CV_CALIB_FIX_K3 |
					   CV_CALIB_FIX_K4 |
					   CV_CALIB_FIX_K5 |
					   CV_CALIB_FIX_K6);
  return error;
}


double
stereo_calibrate(const std::vector< std::vector<cv::Point2f> >& rgb_corners,
		 const std::vector< std::vector<cv::Point2f> >& depth_corners,
		 cv::Mat1d& rgb_intrinsics, cv::Mat1d& rgb_distortion,
		 cv::Mat1d& depth_intrinsics, cv::Mat1d& depth_distortion,
		 cv::Mat1d& R, cv::Mat1d& T)
{

  std::vector< std::vector<cv::Point3f> > pattern_points;
  calibrationPattern(pattern_points,
		     CB_WIDTH,  CB_HEIGHT, 0.075,
		     depth_corners.size());



    cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
    double error = cv::stereoCalibrate(pattern_points,
				       depth_corners, rgb_corners,
				       depth_intrinsics, depth_distortion,
				       rgb_intrinsics, rgb_distortion,
				       cv::Size(0,0),
				       R, T, E, F,
				       cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-6),
				       CV_CALIB_FIX_INTRINSIC);

    return error;
}


int main(int argc, char* argv[]){

  bool compute_distortions = false;
  float tracking_offset_time = 0.0;
  glm::uvec3 grid_size(16,16,16);
  CMDParser p("sweepfilename outputymlfile");
  p.addOpt("d",-1,"distortions", "compute distortion parameters: default false");
  p.addOpt("i",-1,"interactiveshow", "show chessboards that are used for intrinsic calibration and exit without further computations");
  p.addOpt("g",3,"gridsize", "use this grid dimensions to extract chessboard locations: default 16 16 16");
  p.init(argc,argv);


  if(p.getArgs().size() != 2){
    p.showHelp();
    return -1;
  }

  if(p.isOptSet("d")){
    compute_distortions = true;
  }

  if(p.isOptSet("g")){
    grid_size.x = p.getOptsInt("g")[0];
    grid_size.y = p.getOptsInt("g")[1];
    grid_size.z = p.getOptsInt("g")[2];
  }



  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);

  Checkerboard cb;
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cbs(p.getArgs()[0].c_str(), cfg, false /*do not use undistortion of images here of course!*/);
  cbs.init();
  cbs.filterSamples(tracking_offset_time);
  const std::vector<unsigned> cbs_for_intrinsics = cbs.extractBoardsForIntrinsicsFromValidRanges(grid_size.x,grid_size.y,grid_size.z);

  if(p.isOptSet("i")){
    for(const auto& cb_id : cbs_for_intrinsics){
      std::cerr << "cbs.interactiveShow: " << cb_id << std::endl;
      cbs.interactiveShow(cb_id, cb_id);
    }
    return 0;
  }
  
  
  std::vector< std::vector<cv::Point2f> > rgb_stereo_corners;
  std::vector< std::vector<cv::Point2f> > depth_stereo_corners;

  extract_corners(&cbs, cbs_for_intrinsics, rgb_stereo_corners, depth_stereo_corners);


  cv::Mat1d rgb_intrinsics;
  cv::Mat1d rgb_distortion;
  const double error_rgb = calibrate_camera(rgb_stereo_corners, cfg.size_rgb, rgb_intrinsics, rgb_distortion, compute_distortions);
  std::cout << "error_rgb: " << error_rgb << std::endl;

  cv::Mat1d depth_intrinsics;
  cv::Mat1d depth_distortion;
  const double error_d = calibrate_camera(depth_stereo_corners, cfg.size_d, depth_intrinsics, depth_distortion, compute_distortions);
  std::cout << "error_d: " << error_d << std::endl;

  cv::Mat1d R;
  cv::Mat1d T;

  const double error_s = stereo_calibrate(rgb_stereo_corners, depth_stereo_corners,
  					  rgb_intrinsics, rgb_distortion, depth_intrinsics, depth_distortion,
					  R, T);
  std::cout << "average pixel reprojection error: " << error_s << std::endl;


  // WRITE to .yml

  cv::FileStorage output_file(p.getArgs()[1].c_str(),  CV_STORAGE_WRITE);
  writeMatrix(output_file, "rgb_intrinsics", rgb_intrinsics);
  writeMatrix(output_file, "rgb_distortion", rgb_distortion);
  writeMatrix(output_file, "depth_intrinsics", depth_intrinsics);
  writeMatrix(output_file, "depth_distortion", depth_distortion);
  writeMatrix(output_file, "R", R);
  writeMatrix(output_file, "T", T);

  cv::Mat1i size_matrix(1,2);

  size_matrix(0,0) = cfg.size_rgb.x;
  size_matrix(0,1) = cfg.size_rgb.y;
  writeMatrix(output_file, "rgb_size", size_matrix);

  size_matrix(0,0) = cfg.size_d.x;
  size_matrix(0,1) = cfg.size_d.y;
  writeMatrix(output_file, "depth_size", size_matrix);

  output_file.release();

  // READ again and dump
  cfg.read(p.getArgs()[1].c_str()); // if file exists, then read else apply defaults
  cfg.dump();



  return 0;
}


