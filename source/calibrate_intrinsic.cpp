#include <CMDParser.hpp>
#include <ChessboardSampling.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <OpenCVHelper.hpp>


void
extract_corners(ChessboardSampling* cbs,
		std::vector< std::vector<cv::Point2f> >& color_corners,
		std::vector< std::vector<cv::Point2f> >& depth_corners,
		const float color_offset_time,
		const unsigned stride){

  const std::vector<ChessboardRange>& valid_ranges = cbs->getValidRanges();
  const std::vector<ChessboardViewIR>& cb_irs = cbs->getIRs();
  for(const auto& r : valid_ranges){
    for(unsigned i = r.start; i != r.end; ++i){
      
      if((i % stride) == 0){
	const double time = cb_irs[i].time;
	// interpolate color_chessboard 
	bool valid_rgb = false;
	ChessboardViewRGB cb_rgb_i = cbs->interpolateRGB(time + color_offset_time, valid_rgb);
	if(valid_rgb){
	  // add 35 corners to stereo_corners of cb_rgb_i
	  std::vector<cv::Point2f> corners_rgb;
	  std::vector<cv::Point2f> corners_d;
	  for(unsigned idx = 0; idx < (CB_WIDTH * CB_HEIGHT); ++idx){
	    corners_rgb.push_back(cv::Point2f(cb_rgb_i.corners[idx].u,cb_rgb_i.corners[idx].v));
	    corners_d.push_back(cv::Point2f(cb_irs[i].corners[idx].x,cb_irs[i].corners[idx].y));
	  }
	  color_corners.push_back(corners_rgb);
	  depth_corners.push_back(corners_d);
	}
      }
    }
  }

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
				       rgb_corners, depth_corners,
				       rgb_intrinsics, rgb_distortion,
				       depth_intrinsics, depth_distortion,
				       cv::Size(0,0),
				       R, T, E, F,
				       cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-6),
				       CV_CALIB_FIX_INTRINSIC);
    return error;
}


int main(int argc, char* argv[]){

  bool compute_distortions = false;
  unsigned stride = 30;
  float tracking_offset_time = 0.0;
  float color_offset_time = -0.01;
  CMDParser p("sweepfilename outputymlfile");
  p.addOpt("d",-1,"distortions", "compute distortion parameters: default false");
  p.init(argc,argv);


  if(p.getArgs().size() != 2){
    p.showHelp();
    return -1;
  }

  if(p.isOptSet("d")){
    compute_distortions = true;
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

  ChessboardSampling cbs(p.getArgs()[0].c_str());
  cbs.init();
  cbs.filterSamples(tracking_offset_time);

  
  std::vector< std::vector<cv::Point2f> > rgb_stereo_corners;
  std::vector< std::vector<cv::Point2f> > depth_stereo_corners;

  extract_corners(&cbs, rgb_stereo_corners, depth_stereo_corners, color_offset_time, stride);


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

#if 1

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

  // READ from 23.cv_yml
  {
    
#if 0
    
    cfg.read(p.getArgs()[1].c_str()); // if file exists, then read else apply defaults
    
    // defaults are:
    size_rgb = glm::uvec2(1280, 1080);
    size_d   = glm::uvec2(512, 424);

    principal_rgb = glm::vec2(701.972473, 532.143066);
    principal_d   = glm::vec2(257.009552, 209.077789);

    focal_rgb = glm::vec2(1030.829834, 1030.497070);
    focal_d   = glm::vec2(355.433716, 355.672363);

    eye_d_to_eye_rgb[0][0] = 0.999950;
    eye_d_to_eye_rgb[0][1] = -0.009198;
    eye_d_to_eye_rgb[0][2] = -0.003908;
    eye_d_to_eye_rgb[0][3] = 0.0;

    eye_d_to_eye_rgb[1][0] = 0.009169;
    eye_d_to_eye_rgb[1][1] = 0.999932;
    eye_d_to_eye_rgb[1][2] = -0.007234;
    eye_d_to_eye_rgb[1][3] = 0.0;
 
    eye_d_to_eye_rgb[2][0] = 0.003974;
    eye_d_to_eye_rgb[2][1] = 0.007198;
    eye_d_to_eye_rgb[2][2] = 0.999966;
    eye_d_to_eye_rgb[2][3] = 0.0;

    eye_d_to_eye_rgb[3][0] = -0.051237;
    eye_d_to_eye_rgb[3][1] = 0.000667;
    eye_d_to_eye_rgb[3][2] = 0.000195;
    eye_d_to_eye_rgb[3][3] = 1.0;

    // add distortions_rgb, distortions_d as in KinectCalibrationFile;
    

    cfg.dump();

#endif


    cv::Mat1d rgb_intrinsics;
    cv::Mat1d rgb_distortion;

    cv::Mat1d depth_intrinsics;
    cv::Mat1d depth_distortion;

    cv::Mat1d R;
    cv::Mat1d T;

    cv::FileStorage calibration_file(p.getArgs()[1].c_str(), CV_STORAGE_READ);
    readMatrix(calibration_file, "rgb_intrinsics", rgb_intrinsics);
    readMatrix(calibration_file, "rgb_distortion", rgb_distortion);
    readMatrix(calibration_file, "depth_intrinsics", depth_intrinsics);
    readMatrix(calibration_file, "depth_distortion", depth_distortion);
    readMatrix(calibration_file, "R", R);
    readMatrix(calibration_file, "T", T);
    cv::Mat1i size_mat;
    readMatrix(calibration_file, "rgb_size", size_mat);
    // cv::Size(size_mat(0,0), size_mat(0,1));
    std::cout << "rgb_size: " << size_mat(0,0) << " " << size_mat(0,1) << std::endl;
    readMatrix(calibration_file, "depth_size", size_mat);
    calibration_file.release();
  }

#endif


  return 0;
}


