#include "DataTypes.hpp"

#include <OpenCVHelper.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>

#if 0
  cb.pose_offset[0][0] = 0.999970;
  cb.pose_offset[0][1] = -0.001647;
  cb.pose_offset[0][2] = 0.007582;

  cb.pose_offset[1][0] = 0.001702;
  cb.pose_offset[1][1] = 0.999973;
  cb.pose_offset[1][2] = -0.007256;

  cb.pose_offset[2][0] = -0.007571;
  cb.pose_offset[2][1] = 0.007269;
  cb.pose_offset[2][2] = 0.999944;

  cb.pose_offset[3][0] = -0.003584;
  cb.pose_offset[3][1] = 0.002038;
  cb.pose_offset[3][2] = 0.007816;
#endif



bool
RGBDConfig::read(const char* ymlfilename){

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


  intrinsic_rgb[0] = focal_rgb.x;
  intrinsic_rgb[1] = 0.0f;
  intrinsic_rgb[2] = principal_rgb.x;
  intrinsic_rgb[3] = 0.0f;
  intrinsic_rgb[4] = focal_rgb.y;
  intrinsic_rgb[5] = principal_rgb.y;
  intrinsic_rgb[6] = 0.0f;
  intrinsic_rgb[7] = 0.0f;
  intrinsic_rgb[8] = 1.0f;

  intrinsic_d[0] = focal_d.x;
  intrinsic_d[1] = 0.0f;
  intrinsic_d[2] = principal_d.x;
  intrinsic_d[3] = 0.0f;
  intrinsic_d[4] = focal_d.y;
  intrinsic_d[5] = principal_d.y;
  intrinsic_d[6] = 0.0f;
  intrinsic_d[7] = 0.0f;
  intrinsic_d[8] = 1.0f;

  // add distortions_rgb, distortions_d as in KinectCalibrationFile;
  for(unsigned i = 0; i < 5; ++i){
    distortion_rgb[i] = 0.0f;
    distortion_d[i] = 0.0f;
  }
  // end defaults


  struct stat buffer;   
  if(stat(ymlfilename, &buffer) != 0){
    std::cerr << "INFO: RGBDConfig::read file " << ymlfilename << " not found, using defaults" << std::endl;
    return false; 
  }
  else{
    std::cout << "INFO: RGBDConfig::read parsing file " << ymlfilename << std::endl;
  }


  cv::Mat1d rgb_intrinsics;
  cv::Mat1d rgb_distortion;

  cv::Mat1d depth_intrinsics;
  cv::Mat1d depth_distortion;

  cv::Mat1d R;
  cv::Mat1d T;

  cv::FileStorage calibration_file(ymlfilename, CV_STORAGE_READ);
  readMatrix(calibration_file, "rgb_intrinsics", rgb_intrinsics);

  focal_rgb.x = rgb_intrinsics(0,0);
  focal_rgb.y = rgb_intrinsics(1,1);
  principal_rgb.x = rgb_intrinsics(0,2);
  principal_rgb.y = rgb_intrinsics(1,2);


  readMatrix(calibration_file, "rgb_distortion", rgb_distortion);
  distortion_rgb[0] = rgb_distortion(0,0);
  distortion_rgb[1] = rgb_distortion(0,1);
  distortion_rgb[2] = rgb_distortion(0,2);
  distortion_rgb[3] = rgb_distortion(0,3);

  readMatrix(calibration_file, "depth_intrinsics", depth_intrinsics);
  focal_d.x = depth_intrinsics(0,0);
  focal_d.y = depth_intrinsics(1,1);
  principal_d.x = depth_intrinsics(0,2);
  principal_d.y = depth_intrinsics(1,2);

  readMatrix(calibration_file, "depth_distortion", depth_distortion);
  distortion_d[0] = depth_distortion(0,0);
  distortion_d[1] = depth_distortion(0,1);
  distortion_d[2] = depth_distortion(0,2);
  distortion_d[3] = depth_distortion(0,3);


  //std::cout << "before eye_d_to_eye_rgb: " << eye_d_to_eye_rgb << std::endl;

  readMatrix(calibration_file, "R", R);
  eye_d_to_eye_rgb[0][0] = R(0,0);
  eye_d_to_eye_rgb[0][1] = R(1,0);
  eye_d_to_eye_rgb[0][2] = R(2,0);
  eye_d_to_eye_rgb[0][3] = 0.0;

  eye_d_to_eye_rgb[1][0] = R(0,1);
  eye_d_to_eye_rgb[1][1] = R(1,1);
  eye_d_to_eye_rgb[1][2] = R(2,1);
  eye_d_to_eye_rgb[1][3] = 0.0;
  
  eye_d_to_eye_rgb[2][0] = R(0,2);
  eye_d_to_eye_rgb[2][1] = R(1,2);
  eye_d_to_eye_rgb[2][2] = R(2,2);
  eye_d_to_eye_rgb[2][3] = 0.0;

  readMatrix(calibration_file, "T", T);
  eye_d_to_eye_rgb[3][0] = T(0,0);
  eye_d_to_eye_rgb[3][1] = T(1,0);
  eye_d_to_eye_rgb[3][2] = T(2,0);
  eye_d_to_eye_rgb[3][3] = 1.0;

  //std::cout << "after eye_d_to_eye_rgb: " << eye_d_to_eye_rgb << std::endl;



  cv::Mat1i size_mat;
  readMatrix(calibration_file, "rgb_size", size_mat);
  size_rgb = glm::uvec2(size_mat(0,0), size_mat(0,1));
  readMatrix(calibration_file, "depth_size", size_mat);
  size_d   = glm::uvec2(size_mat(0,0), size_mat(0,1));

  calibration_file.release();


  intrinsic_rgb[0] = focal_rgb.x;
  intrinsic_rgb[1] = 0.0f;
  intrinsic_rgb[2] = principal_rgb.x;
  intrinsic_rgb[3] = 0.0f;
  intrinsic_rgb[4] = focal_rgb.y;
  intrinsic_rgb[5] = principal_rgb.y;
  intrinsic_rgb[6] = 0.0f;
  intrinsic_rgb[7] = 0.0f;
  intrinsic_rgb[8] = 1.0f;

  intrinsic_d[0] = focal_d.x;
  intrinsic_d[1] = 0.0f;
  intrinsic_d[2] = principal_d.x;
  intrinsic_d[3] = 0.0f;
  intrinsic_d[4] = focal_d.y;
  intrinsic_d[5] = principal_d.y;
  intrinsic_d[6] = 0.0f;
  intrinsic_d[7] = 0.0f;
  intrinsic_d[8] = 1.0f;

  return true;
}


void
RGBDConfig::dump(){
  std::cout << "RGBDConfig:" << std::endl;
  std::cout << "size_rgb: " << size_rgb << std::endl
	    << "size_d: " << size_d << std::endl
	    << "focal_rgb: " << focal_rgb << std::endl
	    << "principal_rgb: " << principal_rgb << std::endl
	    << "focal_d: " << focal_d << std::endl
	    << "principal_d: " << principal_d << std::endl
	    << "eye_d_to_eye_rgb: " << eye_d_to_eye_rgb << std::endl
	    << std::endl;
}


bool
Checkerboard::save_pose_offset(const char* filename){
  std::ofstream off(filename, std::ofstream::binary);
  off.write((const char*) glm::value_ptr(pose_offset), sizeof(glm::mat4));
  off.close();
  std::cout << "Checkerboard:: saving pose offset " << pose_offset << std::endl;
  return true;
}

bool
Checkerboard::load_pose_offset(const char* filename){
  std::ifstream iff(filename, std::ifstream::binary);
  iff.read((char*) glm::value_ptr(pose_offset), sizeof(glm::mat4));
  iff.close();
  std::cout << "Checkerboard:: loaded pose offset " << pose_offset << std::endl;
  return true;
}


shape_desc::shape_desc(unsigned a, unsigned b, unsigned c, unsigned d)
  : id(){
  id[0] = a;
  id[1] = b;
  id[2] = c;
  id[3] = d;
}

std::ostream& operator << (std::ostream& o, const shape_desc& sd){
  o << "shape_desc: " << sd.id[0] << ", " << sd.id[1] << ", " << sd.id[2] << ", " << sd.id[3];
  return o;
}



std::ostream& operator << (std::ostream& o, const glm::uvec2& v){
    o << "(" << v.x << "," << v.y << ")";
    return o;
  }

std::ostream& operator << (std::ostream& o, const glm::vec2& v){
    o << "(" << v.x << "," << v.y << ")";
    return o;
  }


std::ostream& operator << (std::ostream& o, const glm::vec3& v){
    o << "(" << v.x << "," << v.y << "," << v.z << ")";
    return o;
  }

std::ostream& operator<< (std::ostream& os, const glm::mat4& m){
  os << "mat4[" << std::fixed << std::endl;
  os << "       ("               << m[0][0] << ", " << m[1][0] << ", " << m[2][0]  << ", " << m[3][0] << ")," << std::endl;
  os << "       ("               << m[0][1] << ", " << m[1][1] << ", " << m[2][1]  << ", " << m[3][1] << ")," << std::endl;
  os << "       ("               << m[0][2] << ", " << m[1][2] << ", " << m[2][2]  << ", " << m[3][2] << ")," << std::endl;
  os << "       ("               << m[0][3] << ", " << m[1][3] << ", " << m[2][3]  << ", " << m[3][3] << ") ]" << std::endl;

  return os;
}


  uv interpolate(const uv& a, const uv& b, float t){
    uv r;
    r.u = (1.0f - t)*a.u + t*b.u;
    r.v = (1.0f - t)*a.v + t*b.v;
    return r;
  }



  xyz interpolate(const xyz& a, const xyz& b, float t){
    xyz r;
    r.x = (1.0f - t)*a.x + t*b.x;
    r.y = (1.0f - t)*a.y + t*b.y;
    r.z = (1.0f - t)*a.z + t*b.z;
    return r;
  }



  /*extern*/
  xyz
  operator* (const float v, const xyz& b){
    xyz res;
    res.x = v * b.x;
    res.y = v * b.y;
    res.z = v * b.z;
    return res;
  }


  /*extern*/
  uv
  operator* (const float v, const uv& b){
    uv res;
    res.u = v * b.u;
    res.v = v * b.v;
    return res;
  }



  /*extern*/
  xyz
  operator+ (const xyz& a, const xyz& b){
    xyz res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
  }

  std::ostream& operator << (std::ostream& o, const xyz& v){
    o << "(" << v.x << "," << v.y << "," << v.z << ")";
    return o;
  }

  /*extern*/
  uv
  operator+ (const uv& a, const uv& b){
    uv res;
    res.u = a.u + b.u;
    res.v = a.v + b.v;
    return res;
  }


  /*extern*/
  uv
  operator- (const uv& a, const uv& b){
    uv res;
    res.u = a.u - b.u;
    res.v = a.v - b.v;
    return res;
  }

  std::ostream& operator << (std::ostream& o, const uv& v){
    o << "(" << v.u << "," << v.v << ")";
    return o;
  }

  /*extern*/
  xyz_d
  operator* (const float v, const xyz_d& b){
    xyz_d res;
    res.x = v * b.x;
    res.y = v * b.y;
    res.z = v * b.z;
    return res;
  }


  /*extern*/
  uv_d
  operator* (const float v, const uv_d& b){
    uv_d res;
    res.u = v * b.u;
    res.v = v * b.v;
    return res;
  }



  /*extern*/
  xyz_d
  operator+ (const xyz_d& a, const xyz& b){
    xyz_d res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
  }


  /*extern*/
  uv_d
  operator+ (const uv_d& a, const uv& b){
    uv_d res;
    res.u = a.u + b.u;
    res.v = a.v + b.v;
    return res;
  }



  CandidateSample::CandidateSample(const float w, const xyz& p_off, const uv& t_off):
    weight(w),
    pos_off(),
    tex_off()
  {
    pos_off = p_off;
    tex_off = t_off;
  }

  CandidateSample::~CandidateSample()
  {}


  /*extern*/
  bool
  operator < (const CandidateSample& a, const CandidateSample& b){
    return a.weight > b.weight;
  }






  /* extern */
  std::ostream&
  operator<< (std::ostream& os, const samplePoint& a)
  {
    os << "samplePoint( " << std::fixed 
       << " depth " << a.depth
       << " tex_color.u " << a.tex_color.u
       << " tex_color.v " << a.tex_color.v
       << " tex_depth.u " << a.tex_depth.u
       << " tex_depth.v " << a.tex_depth.v
       << " pos_offset.x " << a.pos_offset.x
       << " pos_offset.y " << a.pos_offset.y
       << " pos_offset.z " << a.pos_offset.z
       << " tex_offset.u " << a.tex_offset.u
       << " tex_offset.v " << a.tex_offset.v
       << " pos_real " << a.pos_real
       << " quality "  << a.quality
       << ")";


    return os;
  }



  float
  getBilinear(float* data, unsigned width, unsigned height, float x, float y){


    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate indices to access data
    const unsigned idmax = width * height;
    unsigned id00 = std::min( ya * width + xa  , idmax);
    unsigned id10 = std::min( ya * width + xb  , idmax);
    unsigned id01 = std::min( yb * width + xa  , idmax);
    unsigned id11 = std::min( yb * width + xb  , idmax);

    // 1. interpolate between x direction;
    float tmp_ya = w_xa * data[id00] + w_xb * data[id10];
    float tmp_yb = w_xa * data[id01] + w_xb * data[id11];
    // 2. interpolate between y direction;
    float result = w_ya * tmp_ya + w_yb * tmp_yb;
    return result;
  }




xyz getTrilinear(xyz* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate weights and boundaries along z direction
    unsigned za = std::floor(z);
    unsigned zb = std::ceil(z);
    float w_zb = z - za;
    float w_za = 1.0 - w_zb;

    // calculate indices to access data
    const unsigned idmax = width * height * depth;
    unsigned id000 = std::min( za * width * height + ya * width + xa  , idmax);
    unsigned id100 = std::min( za * width * height + ya * width + xb  , idmax);
    unsigned id110 = std::min( za * width * height + yb * width + xb  , idmax);
    unsigned id010 = std::min( za * width * height + yb * width + xa  , idmax);

    unsigned id001 = std::min( zb * width * height + ya * width + xa  , idmax);
    unsigned id101 = std::min( zb * width * height + ya * width + xb  , idmax);
    unsigned id111 = std::min( zb * width * height + yb * width + xb  , idmax);
    unsigned id011 = std::min( zb * width * height + yb * width + xa  , idmax);



    // 1. interpolate between x direction: 4 times;
    xyz   tmp_000_100 = w_xa * data[id000] + w_xb * data[id100];
    xyz   tmp_010_110 = w_xa * data[id010] + w_xb * data[id110];
    xyz   tmp_001_101 = w_xa * data[id001] + w_xb * data[id101];
    xyz   tmp_011_111 = w_xa * data[id011] + w_xb * data[id111];

    // 2. interpolate between y direction: 2 times;

    xyz   tmp_A = w_ya * tmp_000_100 + w_yb * tmp_010_110;
    xyz   tmp_B = w_ya * tmp_001_101 + w_yb * tmp_011_111;

    xyz result = w_za * tmp_A + w_zb * tmp_B;

    return result;
}


uv getTrilinear(uv* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate weights and boundaries along z direction
    unsigned za = std::floor(z);
    unsigned zb = std::ceil(z);
    float w_zb = z - za;
    float w_za = 1.0 - w_zb;

    // calculate indices to access data
    const unsigned idmax = width * height * depth;
    unsigned id000 = std::min( za * width * height + ya * width + xa  , idmax);
    unsigned id100 = std::min( za * width * height + ya * width + xb  , idmax);
    unsigned id110 = std::min( za * width * height + yb * width + xb  , idmax);
    unsigned id010 = std::min( za * width * height + yb * width + xa  , idmax);

    unsigned id001 = std::min( zb * width * height + ya * width + xa  , idmax);
    unsigned id101 = std::min( zb * width * height + ya * width + xb  , idmax);
    unsigned id111 = std::min( zb * width * height + yb * width + xb  , idmax);
    unsigned id011 = std::min( zb * width * height + yb * width + xa  , idmax);



    // 1. interpolate between x direction: 4 times;
    uv   tmp_000_100 = w_xa * data[id000] + w_xb * data[id100];
    uv   tmp_010_110 = w_xa * data[id010] + w_xb * data[id110];
    uv   tmp_001_101 = w_xa * data[id001] + w_xb * data[id101];
    uv   tmp_011_111 = w_xa * data[id011] + w_xb * data[id111];

    // 2. interpolate between y direction: 2 times;

    uv   tmp_A = w_ya * tmp_000_100 + w_yb * tmp_010_110;
    uv   tmp_B = w_ya * tmp_001_101 + w_yb * tmp_011_111;

    uv result = w_za * tmp_A + w_zb * tmp_B;

    return result;
}


glm::vec3 calcMean(const std::vector<glm::vec3>& vecs){

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  for(const auto& v : vecs){
    x += v[0];
    y += v[1];
    z += v[2];
  }
  return glm::vec3(x/vecs.size(), y/vecs.size(), z/vecs.size());
}



void calcMeanSD(std::vector<float>& values, double& mean, double& stdev){

  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  mean = sum / values.size();
  
  const double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
  stdev = std::sqrt(sq_sum / values.size() - mean * mean);
  
}


void calcMeanSDMaxMedian(std::vector<float>& values, double& mean, double& stdev, double& ma, double& median){
  calcMeanSD(values, mean, stdev);
  ma = *(std::max_element(values.begin(), values.end()));

  std::vector<float> values_temp = values;
  size_t size = values_temp.size();
  std::sort(values_temp.begin(), values_temp.end());

  if (size  % 2 == 0)
  {
      median = (values_temp[size / 2 - 1] + values_temp[size / 2]) / 2.0;
  }
  else 
  {
      median = values_temp[size / 2];
  }
}


void calcMeanSD(std::vector<double>& values, double& mean, double& stdev){

  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  mean = sum / values.size();
  
  const double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
  stdev = std::sqrt(sq_sum / values.size() - mean * mean);
  
}


void calcMeanSDMaxMedian(std::vector<double>& values, double& mean, double& stdev, double& ma, double& median){
  calcMeanSD(values, mean, stdev);
  ma = *(std::max_element(values.begin(), values.end()));

  std::vector<double> values_temp = values;
  size_t size = values_temp.size();
  std::sort(values_temp.begin(), values_temp.end());

  if (size  % 2 == 0)
  {
      median = (values_temp[size / 2 - 1] + values_temp[size / 2]) / 2.0;
  }
  else 
  {
      median = values_temp[size / 2];
  }
}





size_t calcNumFrames(std::ifstream& f, size_t fs){
  f.seekg(0,std::ios::end);
  const unsigned number_of_frames = (f.tellg()/fs);
  f.seekg(0, std::ios::beg);
  return number_of_frames;
}

